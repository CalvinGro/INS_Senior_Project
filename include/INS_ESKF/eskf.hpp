/*
Author      - Calvin Grossdot produ
Date        - 9/8/26
Modified    - 9/10/26
Modified    - 9/11/26
Modified    - 9/13/26
Title       - Error State Kalman Filter Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file...
*/

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>
#include <stdint.h>

#pragma once

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
    NominalState nominal_state;
    CovarianceMatrix covariance_matrix;



public:
    void inputMeasurement(Measurement new_measurement);
    NominalState getCurrentState(void);
};