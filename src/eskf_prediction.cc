/*
Author      - Calvin Gross
Start Date  - 9/18/26
Title       - Error State Kalman Filter Prediction Methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the methods to predict/update the state vector and
            covariance matrix based on accelerometer or gyroscope measurements.
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


InsStatus Eskf::applyAccelGyroPredict(uint64_t dt) {
        return applyAccelGyroPredict(dt, cur_state.last_pred, cur_state);
};

InsStatus Eskf::applyAccelGyroPredict(uint64_t dt, const AGInternalSample& ag_sample) {
        return applyAccelGyroPredict(dt, ag_sample, cur_state);
};

InsStatus Eskf::applyAccelGyroPredict(
        uint64_t dt,
        const AGInternalSample& ag_sample, 
        FullState& prev_state
    ) {
        
        // Error Checks
        if (dt == 0) return InsStatus::dt_of_0_for_ag;
        if (!prev_state.escm.allFinite()) return InsStatus::escm_uninit_or_not_finite_ag;
        if (
            !prev_state.ns.angular_position.coeffs().allFinite() ||
            !prev_state.ns.position.allFinite() ||
            !prev_state.ns.velocity.allFinite() ||
            !prev_state.ns.accel_bias.allFinite() ||
            !prev_state.ns.gyro_bias.allFinite() ||
            !std::isfinite(prev_state.ns.baro_bias)
        ) return InsStatus::ns_uninit_or_not_finite_ag;
        if (std::abs(prev_state.ns.angular_position.squaredNorm() - 1.0f) > quat_tolerance){
            return InsStatus::ns_quaternion_not_unit_legth_ag;
        }
        if (!ag_sample.accel.allFinite() || !ag_sample.ang_vel.allFinite()) {
            return InsStatus::sample_not_finite_ag;
        }



        float dt_seconds = static_cast<float>(dt) * 1e-6f;

        // Calculate rotation matrix R for the previous orientation (backward integration).
        // Normalization is performed to prevent tiny rounding errors from growing.
        Eigen::Matrix3f R = prev_state.ns.angular_position.normalized().toRotationMatrix();


        // ___First, propagate the nominal state___
        Eigen::Vector3f rotated_accel = (R * (ag_sample.accel - prev_state.ns.accel_bias));

        Eigen::Vector3f new_pos = prev_state.ns.position + prev_state.ns.velocity * dt_seconds + 0.5f * (rotated_accel + gravity) * dt_seconds * dt_seconds;
        Eigen::Vector3f new_vel = prev_state.ns.velocity + (rotated_accel + gravity) * dt_seconds;

        if (!new_pos.allFinite() ||!new_vel.allFinite()) return InsStatus::ns_overflow_ag;
            
        prev_state.ns.position = new_pos;
        prev_state.ns.velocity = new_vel;

        prev_state.ns.angular_position = (prev_state.ns.angular_position * Eskf::RotationVectorToQuaternion(
            (ag_sample.ang_vel - prev_state.ns.gyro_bias) * dt_seconds)
        ).normalized();


        // ___Second, propagate the covariance matrix___

        // calculate the error-state transition matrix, fx
        Eigen::Matrix<float, 16, 16> fx = Eigen::Matrix<float, 16, 16>::Zero();

        // position
        fx.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        fx.block<3,3>(0,3) = Eigen::Matrix3f::Identity() * dt_seconds;

        // velocity 
        fx.block<3,3>(3,3) = Eigen::Matrix3f::Identity();
        fx.block<3,3>(3,6) = -1 * rotated_accel.asSkewSymmetric().toDenseMatrix() * dt_seconds;
        fx.block<3,3>(3,9) = -1 * R * dt_seconds;
        
        // orientation
        // variance stored in global frame
        fx.block<3,3>(6,6) = Eigen::Matrix3f::Identity();
        fx.block<3,3>(6,12) = -1 * R * dt_seconds;

        // biases
        fx.block<3,3>(9,9) = Eigen::Matrix3f::Identity();
        fx.block<3,3>(12,12) = Eigen::Matrix3f::Identity();
        fx(15,15) = 1;

        Eigen::Matrix<float, 16, 16> perturbation_covar = Eigen::Matrix<float, 16, 16>::Zero();
        perturbation_covar.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * conf_state_vgr.velocity_vgr * dt_seconds;
        perturbation_covar.block<3,3>(6,6) = Eigen::Matrix3f::Identity() * conf_state_vgr.orientation_vgr * dt_seconds;
        perturbation_covar.block<3,3>(9,9) = Eigen::Matrix3f::Identity() * conf_state_vgr.accel_bias_vgr * dt_seconds;
        perturbation_covar.block<3,3>(12,12) = Eigen::Matrix3f::Identity() * conf_state_vgr.gyro_bias_vgr * dt_seconds;
        perturbation_covar(15,15) = conf_state_vgr.baro_bias_vgr * dt_seconds;

        prev_state.escm = fx * prev_state.escm * fx.transpose() + perturbation_covar;

        prev_state.last_pred = ag_sample;

        return InsStatus::ok;
};

}
