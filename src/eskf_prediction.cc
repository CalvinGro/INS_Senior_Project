/*
Author      - Calvin Gross
Date        - 9/18/26
Modified    - 9/28/26
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

namespace ins {

bool Eskf::applyAccelAndGyroPrediction(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const AccelGyroUpdateData& ag_sample, 
        uint64_t dt
    ) {
    
    float dt_seconds = static_cast<float>(dt) * 1e-6f;
    NominalState next_state = {};
    CovarianceMatrix next_covar = CovarianceMatrix::Zero();

    // Calculate rotation matrix R for the previous orientation (backward integration).
    // Normalization is performed to prevent tiny rounding errors from growing.
    Eigen::Matrix3f R = nom_state.angular_position.normalized().toRotationMatrix();


    // ___First, propagate the nominal state___
    Eigen::Vector3f rotated_accel = (R * (ag_sample.accel - nom_state.accel_bias));
    
    next_state.position = nom_state.position + nom_state.velocity * dt_seconds + 0.5f * (rotated_accel + gravity) * dt_seconds * dt_seconds;
    next_state.velocity = nom_state.velocity + (rotated_accel + gravity) * dt_seconds;

    next_state.angular_position = (nom_state.angular_position * Eskf::RotationVectorToQuaternion(
        (ag_sample.ang_vel - nom_state.gyro_bias) * dt_seconds)
    ).normalized();

    next_state.accel_bias = nom_state.accel_bias;
    next_state.gyro_bias = nom_state.gyro_bias;
    next_state.baro_bias = nom_state.baro_bias;


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
    fx.block<3,3>(6,6) = Eigen::Matrix3f::Identity();
    fx.block<3,3>(6,12) = -1 * R * dt_seconds;

    // biases
    fx.block<3,3>(9,9) = Eigen::Matrix3f::Identity();
    fx.block<3,3>(12,12) = Eigen::Matrix3f::Identity();
    fx(15,15) = 1;

    Eigen::Matrix<float, 16, 16> perturbation_covar = Eigen::Matrix<float, 16, 16>::Zero();
    perturbation_covar.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * conf_prediction_noises.velocity_noise_v * dt_seconds * dt_seconds;
    perturbation_covar.block<3,3>(6,6) = Eigen::Matrix3f::Identity() * conf_prediction_noises.orientation_noise_v * dt_seconds * dt_seconds;
    perturbation_covar.block<3,3>(9,9) = Eigen::Matrix3f::Identity() * conf_prediction_noises.accel_bias_noise_v * dt_seconds;
    perturbation_covar.block<3,3>(12,12) = Eigen::Matrix3f::Identity() * conf_prediction_noises.gyro_bias_noise_v * dt_seconds;

    next_covar = fx * covar * fx.transpose() + perturbation_covar;

    nom_state = next_state;
    covar = next_covar;

    return true;
};

}

// bool Eskf::applyAccelPrediction(
//         NominalState& nom_state, 
//         CovarianceMatrix& covar, 
//         const AccelUpdateData& accel_sample, 
//         uint16_t dt
//     ) {

// };


// bool Eskf::applyGyroPrediction(
//         NominalState& nom_state, 
//         CovarianceMatrix& covar, 
//         const GyroUpdateData& gyro_sample, 
//         uint16_t dt
//     ) {

// };