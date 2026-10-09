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

    // the most resent 
    FullState cur_state;

    ConfVarGrowthRates conf_prediction_noises;

    inline static const Eigen::Vector3f gravity = {0, 0, -9.80655};

    // DelayedStateBuffer ds_buffer;

    Measurement out_of_bounds_log[OutBoundsLogSize] = {};
    uint16_t oob_log_i = 0;



    // Extended State Info
    
    // method defined in eskf_prediction.cpp

    InsStatus Eskf::AccelGyroPredict(
        uint64_t dt,
        const AccelGyroUpdateData& ag_sample, 
        FullState& prev_state
    );


    // overloaded prev_state to assume current state
    InsStatus Eskf::AccelGyroPredict(
        uint64_t dt,
        const AccelGyroUpdateData& ag_sample
    );

    // methods defined in eskf_correction.cpp
    bool applyMagCorrection(
        NStateVec& nom_state, 
        ESCovarMtx& covar, 
        const MagCorrectionData& mag_sample, 
        uint64_t dt
    );

    bool applyGnssCorrection(
        NStateVec& nom_state, 
        ESCovarMtx& covar,
        const GnssCorrectionData& gnss_sample,
        uint64_t dt
    );

    bool applyBaroCorrection(
        NStateVec& nom_state, 
        ESCovarMtx& covar, 
        const BaroCorrectionData& baro_sample, 
        uint64_t dt
    );

    // utility eskf methods 
    Eigen::Quaternionf Eskf::RotationVectorToQuaternion(const Eigen::Vector3f& rotation_vector);

    bool Eskf::PropgateStatePreCorrection(NStateVec& nom_state, ESCovarMtx& covar, uint64_t dt);
};

}