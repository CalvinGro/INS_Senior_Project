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

#include "ins_types.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>
#include <stdint.h>

namespace ins {

class Eskf
{
    static const uint16_t OutBoundsLogSize = 50;

    NominalState cur_nominal_state;
    CovarianceMatrix cur_covariance_matrix;
    uint64_t cur_ag_timestamp;

    ConfVarGrowthRates conf_prediction_noises;

    inline static const Eigen::Vector3f gravity = {0, 0, -9.80655};

    DelayedStateBuffer ds_buffer;

    Measurement out_of_bounds_log[OutBoundsLogSize] = {};
    uint16_t oob_log_i = 0;

    // method defined in eskf_prediction.cpp

    bool AccelGyroPredict(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const AccelGyroUpdateData& ag_sample, 
        uint64_t dt
    );

    // methods defined in eskf_correction.cpp
    bool applyMagCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const MagCorrectionData& mag_sample, 
        uint64_t dt
    );

    bool applyGnssCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const GnssCorrectionData& gnss_sample, 
        uint64_t dt
    );

    bool applyBaroCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const BaroCorrectionData& baro_sample, 
        uint64_t dt
    );

    // utility eskf methods 
    Eigen::Quaternionf Eskf::RotationVectorToQuaternion(const Eigen::Vector3f& rotation_vector);
};

}