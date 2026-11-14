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
        loe_overflow,

        // measurement input errors
        invalid_pressure_measurement,
        invalid_gnss_position_measurement,
        invalid_mag_field_measurement,

        // init
        duplicate_ag_timestamps_during_init,
        ag_measurements_out_of_order_during_init,
    };

    enum class DelayStatus {
        out_of_bounds,
        delayed,
        on_time,
    };

    enum class SimStatus {
        ok,
        file_not_open_for_recording_states,
        no_more_measurements_to_apply,
        no_input_file_for_trajectory_building,
        no_output_file_created_for_trajectory_building,
        cant_apply_malformed_measurement,
        get_failed,
    };

    enum class InsPhase {
        uninitialized,
        initializing,
        running,
    };

    
    struct NStateVec {
        Eigen::Vector3f position = Eigen::Vector3f::Zero();
        Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
        Eigen::Quaternionf angular_position = Eigen::Quaternionf::Identity();
        Eigen::Vector3f accel_bias = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro_bias = Eigen::Vector3f::Zero(); // stored in bf coordinates (rad^2) / (s^2)
        float baro_bias = 0.0f; // stored altitude (not pressure) bias in meters
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
        Eigen::Vector3f offset = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity(); // in sensor frame
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Vector3f mag_var = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Vector3f mag_field_enu = {0.0f, 40.0f, 0.0f}; // vector in enu of magnetic field from ref frame
    };

    struct AccelConfiguration {
        Eigen::Vector3f offset = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity(); // in sensor frame
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Vector3f accel_vgr = Eigen::Vector3f::Zero();  // in sensor frame pre-correction
        Eigen::Vector3f bias_vgr = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Vector3f initial_residual_bias_var = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
    };

    struct GyroConfiguration {
        Eigen::Vector3f offset = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Matrix3f correction = Eigen::Matrix3f::Identity(); // in sensor frame
        Eigen::Matrix3f sensor_to_body = Eigen::Matrix3f::Identity();
        Eigen::Vector3f ang_vel_vgr = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Vector3f bias_vgr = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
    };

    struct BaroConfiguration {
        float var = 0; // pressure variance (Pa^2)
        float bias_vgr = 0; // in m^2/s
    };

    struct GnssConfiguration {
        Eigen::Vector3f antenna_lever_arm = Eigen::Vector3f::Zero(); // lever arm in body frame
        Eigen::Vector3f default_var = Eigen::Vector3f::Zero();
    };

    struct EskfSettings {
        Eigen::Matrix3f gyro_correction = Eigen::Matrix3f::Identity(); // rotation from sensor to body and correction
        Eigen::Vector3f gyro_offset = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Matrix3f ang_vel_vgr = Eigen::Matrix3f::Zero(); // in body frame post-correction
        Eigen::Matrix3f gyro_bias_vgr = Eigen::Matrix3f::Zero(); // in body frame post-correction

        Eigen::Matrix3f accel_correction = Eigen::Matrix3f::Identity(); // rotation from sensor to body and correction
        Eigen::Vector3f accel_offset = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Matrix3f accel_vgr = Eigen::Matrix3f::Zero();  // in sensor frame pre-correction
        Eigen::Matrix3f accel_bias_vgr = Eigen::Matrix3f::Zero(); // in sensor frame pre-correction
        
        Eigen::Matrix3f mag_correction = Eigen::Matrix3f::Identity(); // rotation from sensor to body and correction
        Eigen::Vector3f mag_offset = Eigen::Vector3f::Zero(); // in sensor frame pre-correction
        Eigen::Vector3f mag_field_enu = Eigen::Vector3f::Identity(); // The magnetic field in ENU frame
        Eigen::Matrix3f mag_var = Eigen::Matrix3f::Zero(); // in body frame post-correction

        float baro_offset = 0.0f; // altitude offset in meters
        float baro_var = 0.0f; // baro variance in meters^2 (based off linearization at ref frame / init alt)
        float baro_bias_vgr = 0.0f; // baro bias vgr in meters^2 / second

        Eigen::Vector3f antenna_lever_arm = Eigen::Vector3f::Zero(); // lever arm in body frame
        Eigen::Vector3f default_gnss_var = Eigen::Vector3f::Zero(); // enu frame
        Eigen::Vector3d origin_ecef; // the starting point / reference frame in meters
        Eigen::Matrix3d ecef_to_enu; // rotation matrix from ecef to enu at reference framed
    };

    struct InitAccumulationState {
        InsPhase init_phase = InsPhase::uninitialized;
        uint64_t first_timestamp = 0;
        uint64_t prev_ag_timestamp = 0; // used for vgr calculations
        AGInternalSample last_ag;

        uint16_t ag_count = 0;
        Eigen::Vector3f accel_mean = Eigen::Vector3f::Zero(); // sensor frame (no correction or offset applied yet)
        Eigen::Vector3f ang_vel_mean = Eigen::Vector3f::Zero(); // sensor frame (no correction or offset applied yet)
        Eigen::Vector3f accel_var = Eigen::Vector3f::Zero(); // sensor frame (no correction or offset applied yet)
        Eigen::Vector3f ang_vel_var = Eigen::Vector3f::Zero(); // sensor frame (no correction or offset applied yet)     

        uint16_t mag_count = 0; 
        Eigen::Vector3f mag_field_mean = Eigen::Vector3f::Zero(); // sensor frame (no correction or offset applied yet)
        Eigen::Vector3f mag_field_var = Eigen::Vector3f::Zero(); // sensor frame (no correction or offset applied yet)

        uint16_t baro_count = 0;
        float baro_pres_mean = 0.0f; // pressure in Pa
        float baro_pres_var = 0.0f; // pressure in Pa^2

        uint16_t gnss_count = 0;
        Eigen::Vector3d ecef_pos_mean = Eigen::Vector3f::Zero(); 
        Eigen::Vector3f enu_pos_var = Eigen::Vector3f::Zero(); // relative to enu coordinates at the system's location
    };

    // this is in sensor frame
    struct SimV1Configuration {
        float accel_vgr_x, accel_vgr_y, accel_vgr_z;
        float gyro_vgr_x, gyro_vgr_y, gyro_vgr_z;
        float mag_var_x, mag_var_y, mag_var_z;
        float baro_var_z;
        float accel_bias_vgr_x, accel_bias_vgr_y, accel_bias_vgr_z;
        float gyro_bias_vgr_x, gyro_bias_vgr_y, gyro_bias_vgr_z;
        float baro_bias_vgr_z;
        float gnss_default_var_x, gnss_default_var_y, gnss_default_var_z;
    };

    inline SimV1Configuration default_config = {
        // Accelerometer white-noise intensity: m^2/s^3
        .accel_vgr_x = 3.08e-7f,
        .accel_vgr_y = 3.08e-7f,
        .accel_vgr_z = 3.08e-7f,

        // Gyroscope white-noise intensity: rad^2/s
        .gyro_vgr_x = 2.44e-9f,
        .gyro_vgr_y = 2.44e-9f,
        .gyro_vgr_z = 2.44e-9f,

        // Magnetometer sample variance: uT^2
        .mag_var_x = 0.1024f,
        .mag_var_y = 0.1024f,
        .mag_var_z = 0.1681f,

        // Barometer altitude sample variance: m^2
        .baro_var_z = 0.0367f,

        // Assumed accelerometer bias random walk: m^2/s^5
        .accel_bias_vgr_x = 1.0e-6f,
        .accel_bias_vgr_y = 1.0e-6f,
        .accel_bias_vgr_z = 1.0e-6f,

        // Assumed gyroscope bias random walk: rad^2/s^3
        .gyro_bias_vgr_x = 1.0e-8f,
        .gyro_bias_vgr_y = 1.0e-8f,
        .gyro_bias_vgr_z = 1.0e-8f,

        // Assumed barometer bias random walk: m^2/s
        .baro_bias_vgr_z = 1.0e-4f,

        .gnss_default_var_x = 4.5084,
        .gnss_default_var_y = 4.5084,
        .gnss_default_var_z = 18.0337
    };
};
