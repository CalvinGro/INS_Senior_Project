/*
Author      - Calvin Gross
Date        - 9/27/26
Title       - Integrated Navigation System Types
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header file that declares all the types that will be used
            universally within my INS project. It identifies them under a namespace.
*/



#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>
#include <stdint.h>


namespace ins {
    
    // Definitions of Measurement Types
    struct AccelGyroUpdateData {
        Eigen::Vector3f accel;
        Eigen::Vector3f ang_vel;
    };

    struct MagCorrectionData {
        Eigen::Vector3f ang_pos;
    };

    struct GnssCorrectionData {
        Eigen::Vector3f pos;
    };

    struct BaroCorrectionData {
        float z_pos;
    };

    struct Measurement {
        std::variant<
            AccelGyroUpdateData,
            MagCorrectionData,
            GnssCorrectionData,
            BaroCorrectionData
        > data;
        uint64_t timestamp;
    };

    struct NominalState {
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
        Eigen::Quaternionf angular_position = Eigen::Quaternionf::Identity();
        Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
        float baro_bias = 0.0f;
    };

    using CovarianceMatrix = Eigen::Matrix<float, 16, 16>;

    enum class EsfkStatus {
        success,
        eskf_uninitialized,
        invalid_measurment,
        error
    };

    // Variance Growth Rates
    struct ConfVarGrowthRates {
        Eigen::Vector3f velocity_vgr = Eigen::Vector3f::Zero();
        Eigen::Vector3f orientation_vgr = Eigen::Vector3f::Zero();
        Eigen::Vector3f accel_bias_vgr = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro_bias_vgr = Eigen::Vector3f::Zero();
        float baro_bias_vgr = 0.0f;
    };
}
