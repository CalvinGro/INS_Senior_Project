/*
Author      - Calvin Grossdot produ
Date        - 9/8/26
Modified    - 9/10/26
Modified    - 9/11/26
Modified    - 9/13/26
Modified    - 9/18/26
Title       - Error State Kalman Filter Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file...
*/


#pragma once


#include "delayed_state_buffer.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>
#include <stdint.h>


class Eskf
{
public:

    // Definitions of Measurement Types
    struct AccelUpdateData {
        Eigen::Vector3f accel;
    };

    struct GyroUpdateData {
        Eigen::Vector3f ang_vel;
    };

    struct MagCorrectionData {
        Eigen::Vector3f ang_pos;
    };

    struct GnssCorrectionData {
        Eigen::Vector2f xy_pos;
    };

    struct BaroCorrectionData {
        float z_pos;
    };

    struct Measurement {
        std::variant<
            AccelUpdateData,
            GyroUpdateData,
            MagCorrectionData,
            GnssCorrectionData,
            BaroCorrectionData
        > data;
        uint64_t timestamp;
    };

    struct NominalState {
        Eigen::Vector3f position;
        Eigen::Vector3f velocity;
        Eigen::Quaternionf angular_position;
        Eigen::Vector3f accel_bias;
        Eigen::Vector3f gyro_bias;
        float baro_bias;
    };

    struct CovarianceMatrix {
        Eigen::Vector3f cv_position;
        Eigen::Vector3f cv_velocity;
        Eigen::Vector3f cv_angular_position;
        Eigen::Vector3f cv_accel_bias;
        Eigen::Vector3f cv_gyro_bias;
        float cv_baro_bias;
    };



private:
    NominalState cur_nominal_state;
    CovarianceMatrix cur_covariance_matrix;

    DelayedStateBuffer ds_buffer;


    // methods defined in eskf_prediction.cpp
    bool apply_accel_prediction(const AccelUpdateData& accel_sample, uint16_t dt);

    bool apply_gyro_prediction(const GyroUpdateData& gyro_sample, uint16_t dt);

    
    // methods defined in eskf_correction.cpp
    bool apply_mag_correction(const MagCorrectionData& mag_sample, uint16_t dt);

    bool apply_gnss_correction(const GnssCorrectionData& gnss_sample, uint16_t dt);

    bool apply_baro_correction(const BaroCorrectionData& baro_sample, uint16_t dt);

public:


    // methods defined in eskf.cpp
    bool initEskf(void);

    bool applyMeasurement(const Measurement& new_measurement);

    bool getCurrentState(NominalState& nominal_state);

};