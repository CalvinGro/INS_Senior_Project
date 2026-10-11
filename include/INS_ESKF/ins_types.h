/*
Author      - Calvin Gross
Start Date  - 9/27/26
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
        Eigen::Vector3f mag_field_body;
    };

    struct GnssCorrectionData {
        // pos in ENU
        Eigen::Vector3f pos;
        // measurement noise
        Eigen::Vector3f gnss_noise;
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

    // enum of every possible status/error code in Ins, Eskf, or Delayed State Filters
    enum class InsStatus {
        ok,

        // accel + gyro prediction errors
        dt_of_0_for_ag,
        escm_uninit_or_not_finite_ag,
        ns_uninit_or_not_finite_ag,
        ns_quaternion_not_unit_legth_ag,
        ns_overflow_ag,
        sample_not_finite_ag,

        // gnss errors
        escm_uninit_or_not_finite_gnss,
        ns_uninit_or_not_finite_gnss,
        ns_quaternion_not_unit_legth_gnss,
        sample_not_finite_gnss,

        // barometer errors
        escm_uninit_or_not_finite_b,
        ns_uninit_or_not_finite_b,
        ns_quaternion_not_unit_legth_b,
        sample_not_finite_b,

        // magnetometer errors
        escm_uninit_or_not_finite_m,
        ns_uninit_or_not_finite_m,
        ns_quaternion_not_unit_legth_m,
        sample_not_finite_m
    };


    struct NStateVec {
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
        Eigen::Quaternionf angular_position = Eigen::Quaternionf::Identity();
        Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero();
        float baro_bias = 0.0f;
    };


    using ESCovarMtx = Eigen::Matrix<float, 16, 16>;

    // Full State Info (contains everything needed to propagate any measurement)
    struct FullState {
        NStateVec ns;                  // nominal-state vector
        ESCovarMtx escm;               // error-state covariance matrix
        AccelGyroUpdateData last_pred; // last accel + gyro prediction (zero order holding for corrections)
        uint64_t last_timestamp;       // time associated with lastest measurement (correction or prediction)
    };


    // Variance Growth Rates
    struct ConfVarGrowthRates {
        Eigen::Vector3f velocity_vgr = Eigen::Vector3f::Zero();
        Eigen::Vector3f orientation_vgr = Eigen::Vector3f::Zero();
        Eigen::Vector3f accel_bias_vgr = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro_bias_vgr = Eigen::Vector3f::Zero();
        float baro_bias_vgr = 0.0f;
    };

    struct MeasurementVars {
        Eigen::Vector3f magnetometer_var = Eigen::Vector3f::Zero();
        float baro_var;
    };

    struct MagnetometerCalibration {
        Eigen::Vector3f offset = Eigen::Vector3f::Zero();
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f mag_field_enu = Eigen::Vector3f::Zero();
    };
}
