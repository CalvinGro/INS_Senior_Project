/*
Author      - Calvin Gross
Start Date  - 9/18/26
Title       - Error State Kalman Filter Correction Methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the methods to correct the state vector and covariance
            matrix based on gnss, magnetometer, or barometer measurements.
*/


#include "ins_types.h"
#include "eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>
#include <stdint.h>
#include <cmath>

#define quat_tolerance 1e-5f

namespace ins {

InsStatus Eskf::applyMagCorrection(
        uint64_t dt,
        const MagInternalSample& mag_sample,
        FullState& prev_state
    ) {
        // Error Checks
        if (!prev_state.escm.allFinite()) return InsStatus::escm_uninit_or_not_finite_m;
        if (
            !prev_state.ns.angular_position.coeffs().allFinite() ||
            !prev_state.ns.position.allFinite() ||
            !prev_state.ns.velocity.allFinite() ||
            !prev_state.ns.accel_bias.allFinite() ||
            !prev_state.ns.gyro_bias.allFinite() ||
            !std::isfinite(prev_state.ns.baro_bias)
        ) return InsStatus::ns_uninit_or_not_finite_m;
        if (std::abs(prev_state.ns.angular_position.squaredNorm() - 1.0f) > quat_tolerance){
            return InsStatus::ns_quaternion_not_unit_legth_m;
        }
        if (!mag_sample.mag_field_body.allFinite()) {
            return InsStatus::sample_not_finite_m;
        }

        
        // Propagate the current state to the most recent timestamp using the most recent prediction.
        if (dt != 0) {
            InsStatus propagation_status = AccelGyroPredict(dt, prev_state.last_pred, prev_state);
            if (propagation_status != InsStatus::ok) return propagation_status;
        }

        Eigen::Matrix3f R = prev_state.ns.angular_position.normalized().toRotationMatrix();
        
        // take predicted magnetic field in enu frame and rotate it into body frame for comparison
        Eigen::Vector3f predicted_mag_field = R.transpose() * conf_mag_cal.mag_field_enu;


        // calculate innovation 
        Eigen::Vector3f innovation =  mag_sample.mag_field_body - predicted_mag_field;

        // The Measurement Noise Covariance (V), built from the passed in uncertainties
        Eigen::Matrix3f V = conf_mea_vars.magnetometer_var.asDiagonal();

        // The Measurement Jacobian (H), translates orientation error into measurement error (linearization)
        Eigen::Matrix<float,3,16> H = Eigen::Matrix<float,3,16>::Zero();
        H.block<3,3>(0,6) = R.transpose() * conf_mag_cal.mag_field_enu.asSkewSymmetric();

        // Calculate Kalman Gain - The matrix to convert the residual into the es correction
        Eigen::Matrix<float,16,3> K = prev_state.escm * H.transpose() * (H * prev_state.escm * H.transpose() + V).inverse();

        // Calculate Error-State - the observed change for the nominal-state from the measurement
        Eigen::Vector<float,16> es = K * innovation;


        // ADD CHI SQUARE GATING


        // Calculate the new error-state covariance matrix
        prev_state.escm = (Eigen::Matrix<float,16,16>::Identity() - K * H) * prev_state.escm;

        // Inject the error-state back into the nominal-state
        CorrectionInjection(es, prev_state.ns);
        
        // Covariance Reset Jacobian (G) - Used to convert the error-state covariance matrix's 
        // orientation variance to be expressed in terms of the new nominal state.
        Eigen::Matrix<float,16,16> G = Eigen::Matrix<float,16,16>::Identity();
        G.block<3,3>(6,6) += (0.5f * es.segment<3>(6)).asSkewSymmetric().toDenseMatrix();

        // calculate new covariance orientation (represent in terms of the new nominal-state)
        prev_state.escm = (G * prev_state.escm * G.transpose()).eval();
        
        return InsStatus::ok;
};

/*  @brief This method applies a gnss correction to the current nom-state and es-covar.
    If no nom_state and covar are passed in (as done for FR) it assumes the correction
    is for the current state. 

    Errors Handled: Missing GNSS variance, missing Nom-state and uninitilized, Missing Covar
    and uninitilized, and Chi-Squared measurement verification with a 99% gate.

    TD - add post-calc pre-injection error checks to validate new corrections
       - add Chi-squared measurement gating
       - add more covariance validation: symetric

    @param[in] dt - change in time since last measurement of any kind
    @param[in] gnss_sample - gnss sample in ENU (INS handles Geodetic -> ECEF -> ENU)
    @param[in,out] FullState - nominal state and error-state covariance getting corrected; 
                               they default to the current full state.
 */
InsStatus Eskf::applyGnssCorrection(
        uint64_t dt,
        const GnssInternalSample& gnss_sample
    ) {
        return applyGnssCorrection(dt, gnss_sample, cur_state);
};

InsStatus Eskf::applyGnssCorrection(
        uint64_t dt,
        const GnssInternalSample& gnss_sample,
        FullState& prev_state
    ) {

        // Error Checks
        if (!prev_state.escm.allFinite()) return InsStatus::escm_uninit_or_not_finite_gnss;
        if (
            !prev_state.ns.angular_position.coeffs().allFinite() ||
            !prev_state.ns.position.allFinite() ||
            !prev_state.ns.velocity.allFinite() ||
            !prev_state.ns.accel_bias.allFinite() ||
            !prev_state.ns.gyro_bias.allFinite() ||
            !std::isfinite(prev_state.ns.baro_bias)
        ) return InsStatus::ns_uninit_or_not_finite_gnss;
        if (std::abs(prev_state.ns.angular_position.squaredNorm() - 1.0f) > quat_tolerance){
            return InsStatus::ns_quaternion_not_unit_legth_gnss;
        }
        if (!gnss_sample.pos.allFinite() || !gnss_sample.gnss_noise.allFinite()) {
            return InsStatus::sample_not_finite_gnss;
        }
        
        // Propagate the current state to the most recent timestamp using the most recent prediction.
        if (dt != 0) {
            InsStatus propagation_status = AccelGyroPredict(dt, prev_state.last_pred, prev_state);
            if (propagation_status != InsStatus::ok) return propagation_status;
        }

        // The Measurement Noise Covariance (V), built from the passed in uncertainties
        Eigen::Matrix3f V = gnss_sample.gnss_noise.asDiagonal();

        // The Measurement Jacobian (H) is just a selection matrix for gnss
        Eigen::Matrix<float,3,16> H = Eigen::Matrix<float,3,16>::Zero();
        H.block<3,3>(0,0).setIdentity();

        // Calculate Kalman Gain - The matrix to convert the residual into the es correction
        Eigen::Matrix<float,16,3> K = prev_state.escm * H.transpose() * (H * prev_state.escm * H.transpose() + V).inverse();

        // Calculate Error-State - the observed change for the nominal-state from the measurement
        Eigen::Vector<float,16> es = K * (gnss_sample.pos - prev_state.ns.position);


        // ADD CHI SQUARE GATING


        // Calculate the new error-state covariance matrix
        prev_state.escm = (Eigen::Matrix<float,16,16>::Identity() - K * H) * prev_state.escm;

        // Inject the error-state back into the nominal-state
        CorrectionInjection(es, prev_state.ns);
        
        // Covariance Reset Jacobian (G) - Used to convert the error-state covariance matrix's 
        // orientation variance to be expressed in terms of the new nominal state.
        Eigen::Matrix<float,16,16> G = Eigen::Matrix<float,16,16>::Identity();
        G.block<3,3>(6,6) += (0.5f * es.segment<3>(6)).asSkewSymmetric().toDenseMatrix();

        // calculate new covariance orientation (represent in terms of the new nominal-state)
        prev_state.escm = (G * prev_state.escm * G.transpose()).eval();
        
        return InsStatus::ok;
};

/*  @brief This method applies a barometer correction to the current nom-state and es-covar.
    If no nom_state and covar are passed in (as done for FR) it assumes the correction
    is for the current state. 

    Errors Handled: Missing baro variance, missing Nom-state and uninitilized, Missing Covar
    and uninitilized, and Chi-Squared measurement verification with a 99% gate.

    TD - add post-calc pre-injection error checks to validate new corrections
       - add Chi-squared measurement gating
       - add more covariance validation: symetric

    @param[in] dt - change in time since last measurement of any kind
    @param[in] baro_sample - barometer sample relative to initial frame
    @param[in,out] FullState - nominal state and error-state covariance getting corrected; 
                               they default to the current full state.
 */
InsStatus Eskf::applyBaroCorrection(
        uint64_t dt,
        const BaroInternalSample& baro_sample
    ) {
        return applyBaroCorrection(dt, baro_sample, cur_state);
};

InsStatus Eskf::applyBaroCorrection(
        uint64_t dt,
        const BaroInternalSample& baro_sample,
        FullState& prev_state
    ) {
                
        // Error Checks
        if (!prev_state.escm.allFinite()) return InsStatus::escm_uninit_or_not_finite_b;
        if (
            !prev_state.ns.angular_position.coeffs().allFinite() ||
            !prev_state.ns.position.allFinite() ||
            !prev_state.ns.velocity.allFinite() ||
            !prev_state.ns.accel_bias.allFinite() ||
            !prev_state.ns.gyro_bias.allFinite() ||
            !std::isfinite(prev_state.ns.baro_bias)
        ) return InsStatus::ns_uninit_or_not_finite_b;
        if (std::abs(prev_state.ns.angular_position.squaredNorm() - 1.0f) > quat_tolerance){
            return InsStatus::ns_quaternion_not_unit_legth_b;
        }
        if (!std::isfinite(baro_sample.z_pos)) {
            return InsStatus::sample_not_finite_b;
        }
        
        // Propagate the current state to the most recent timestamp using the most recent prediction.
        if (dt != 0) {
            InsStatus propagation_status = AccelGyroPredict(dt, prev_state.last_pred, prev_state);
            if (propagation_status != InsStatus::ok) return propagation_status;
        }

        // The Measurement Jacobian (H) is just a selection vector for the barometer, which includes barometer bias
        Eigen::Vector<float,16> H = Eigen::Vector<float,16>::Zero();
        H[2] = 1;
        H[15] = 1;

        // Calculate Kalman Gain - The matrix to convert the residual into the es correction
        Eigen::Vector<float,16> K = prev_state.escm * H / ((H.transpose() * prev_state.escm * H)[0] + conf_mea_vars.baro_var);

        // Calculate Error-State - the observed change for the nominal-state from the measurement
        Eigen::Vector<float,16> es = K * (baro_sample.z_pos - (prev_state.ns.position[2] + prev_state.ns.baro_bias));


        // ADD CHI SQUARE GATING


        // Calculate the new error-state covariance matrix
        prev_state.escm = ((Eigen::Matrix<float,16,16>::Identity() - K * H.transpose()) * prev_state.escm).eval();

        // Inject the error-state back into the nominal-state
        CorrectionInjection(es, prev_state.ns);
        
        // Covariance Reset Jacobian (G) - Used to convert the error-state covariance matrix's 
        // orientation variance to be expressed in terms of the new nominal state.
        Eigen::Matrix<float,16,16> G = Eigen::Matrix<float,16,16>::Identity();
        G.block<3,3>(6,6) += (0.5f * es.segment<3>(6)).asSkewSymmetric().toDenseMatrix();

        // calculate new covariance orientation (represent in terms of the new nominal-state)
        prev_state.escm = (G * prev_state.escm * G.transpose()).eval();
        
        return InsStatus::ok;
};

}