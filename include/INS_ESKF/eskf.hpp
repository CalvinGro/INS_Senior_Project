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
    struct AccelUpdateSample {
        Eigen::Vector3f accel;
        uint64_t timestamp;
    };

    struct GyroUpdateSample {
        Eigen::Vector3f ang_vel;
        uint64_t timestamp;
    };

    struct MagCorrectionSample {
        Eigen::Vector3f ang_pos;
        uint64_t timestamp;
    };

    struct GnssCorrectionSample {
        Eigen::Vector2f xy_pos;
        uint64_t timestamp;
    };

    struct BaroCorrectionSample {
        float z_pos;
        uint64_t timestamp;
    };

    using Measurement = std::variant<
        AccelUpdateSample,
        GyroUpdateSample,
        MagCorrectionSample,
        GnssCorrectionSample,
        BaroCorrectionSample
    >;


private:
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

    NominalState nominal_state;
    CovarianceMatrix covariance_matrix;



public:
    void input_gyro_sample(GyroUpdate cur_gyro);
    void input_accel_sample(AccelUpdate cur_accel);

    void input_baro_sample(BaroCorrection cur_baro);
    void input_gnss_sample(GnssCorrection cur_gnss);
    void input_mag_sample(MagCorrection cur_mag);
};