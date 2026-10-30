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
    
    // Definitions of Internal Measurement Types
    struct AGInternalSample {
        Eigen::Vector3f accel;
        Eigen::Vector3f ang_vel;
    };

    struct MagInternalSample {
        Eigen::Vector3f mag_field_body;
    };

    struct GnssInternalSample {
        // pos in ENU
        Eigen::Vector3f pos;
        // measurement noise
        Eigen::Vector3f gnss_noise;
    };

    struct BaroInternalSample {
        float z_pos;
    };

    struct InternalMeasurement {
        std::variant<
            AGInternalSample,
            MagInternalSample,
            GnssInternalSample,
            BaroInternalSample
        > data;
        uint64_t timestamp;
    };


    // Definitions of External Measurement Types
    struct AGExternalV1Sample {
        float accel_x, accel_y, accel_z;
        float ang_vel_x, ang_vel_y, ang_vel_z;
        uint64_t timestamp;
    };

    struct MagExternalV1Sample {
        float ang_pos_x, ang_pos_y, ang_pos_z;
        uint64_t timestamp;
    };

    struct GnssExternalV1Sample {
        float pos_x, pos_y, pos_z;
        float gnss_var_x, gnss_var_y, gnss_var_z;
        uint64_t timestamp;
    };

    struct BaroExternalV1Sample {
        float z_pos;
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
        sample_not_finite_m,

        // apply measurment errors
        timestamps_out_of_order_applyMeasurement,

        // free-list errors
        all_nodes_full,
        attempted_to_acquire_invalid_index,
        attempted_to_release_invalid_index,

        // delayed state FR errors
        recorded_states_uninit,
        bi_search_failed_in_findRecalcCheckpoint,
        indexed_past_last_measurement,
        invalid_state_index_removeLastState,
        unlinked_state_removeLastState,
        bi_search_found_wrong_state,
        failed_to_add_first_state,
        measurement_out_of_fr_range,

        // error overflow
        loe_overflow
    };

    enum class DelayStatus {
        out_of_bounds,
        delayed,
        on_time
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
        AGInternalSample last_pred;    // last accel + gyro prediction (zero order holding for corrections)
        uint64_t last_timestamp;       // time associated with lastest measurement (correction or prediction)
    };


    struct MagConfiguration {
        Eigen::Vector3f offset = Eigen::Vector3f::Zero();
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f bf_covar = Eigen::Matrix3f::Identity();
        Eigen::Vector3f mag_field_enu = Eigen::Vector3f::Zero();
    };

    struct AccelConfiguration {
        Eigen::Vector3f offset = Eigen::Vector3f::Zero();
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f bf_covar = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f bias_bf_covar = Eigen::Matrix3f::Identity();
    };

    struct GyroConfiguration {
        Eigen::Vector3f offset = Eigen::Vector3f::Zero();
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f bf_covar = Eigen::Matrix3f::Identity();
        Eigen::Matrix3f bias_bf_covar = Eigen::Matrix3f::Identity();
    };

    struct BaroConfiguration {
        float var = 0;
        float bias_var = 0;
    };

    struct GnssConfiguration {
        Eigen::Vector3f antenna_lever_arm = Eigen::Vector3f::Zero();
        Eigen::Matrix3f default_covar = Eigen::Matrix3f::Identity();
    };

    struct SimV1Configuration {
        float accel_vgr_x, accel_vgr_y, accel_vgr_z;
        float gyro_vgr_x, gyro_vgr_y, gyro_vgr_z;
        float mag_vgr_x, mag_vgr_y, mag_vgr_z;
        float baro_vgr_z;
        float accel_bias_vgr_x, accel_bias_vgr_y, accel_bias_vgr_z;
        float gyro_bias_vgr_x, gyro_bias_vgr_y, gyro_bias_vgr_z;
        float baro_bias_bgr_z;
    };
}
