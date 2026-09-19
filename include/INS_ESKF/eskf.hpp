/*
Author      - Calvin Gross
Date        - 9/8/26
Modified    - 9/10/26
Modified    - 9/11/26
Modified    - 9/13/26
Modified    - 9/18/26
Modified    - 9/19/26
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

    enum class MeasurementTypes {
        accel,
        gyro,
        mag,
        gnss,
        baro
    };

    // Definitions of Measurement Types
    struct AccelGyroUpdateData {
        Eigen::Vector3f accel;
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
            AccelGyroUpdateData,
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
    static const uint16_t OutBoundsLogSize = 50;

    NominalState cur_nominal_state;
    CovarianceMatrix cur_covariance_matrix;

    DelayedStateBuffer ds_buffer;

    Measurement out_of_bounds_log[OutBoundsLogSize] = {};
    uint16_t oob_log_i = 0;

    // method defined in eskf_prediction.cpp

    bool applyAccelAndGyroPrediction(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const AccelGyroUpdateData& ag_sample, 
        uint16_t dt
    );

    // methods defined in eskf_correction.cpp
    bool applyMagCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const MagCorrectionData& mag_sample, 
        uint16_t dt
    );

    bool applyGnssCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const GnssCorrectionData& gnss_sample, 
        uint16_t dt
    );

    bool applyBaroCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const BaroCorrectionData& baro_sample, 
        uint16_t dt
    );


public:

    // methods defined in eskf.cpp
    bool initEskf(void);

    bool applyMeasurement(const Measurement& new_measurement);

    bool getCurrentState(NominalState& nominal_state);

};