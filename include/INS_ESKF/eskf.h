/*
Author      - Calvin Gross
Start Date  - 9/8/26
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

    // overloaded - Used when updating the current states
    InsStatus Eskf::AccelGyroPredict(
        uint64_t dt,
        const AccelGyroUpdateData& ag_sample
    );

    // overloaded - Used when propagating to a given timestamp before correction
    InsStatus Eskf::AccelGyroPredict(uint64_t dt);


    // methods defined in eskf_correction.cpp
    InsStatus applyMagCorrection(
        NStateVec& nom_state, 
        ESCovarMtx& covar, 
        const MagCorrectionData& mag_sample, 
        uint64_t dt
    );

    InsStatus applyGnssCorrection(
        uint64_t dt,
        const GnssCorrectionData& gnss_sample,
        FullState& prev_state
    );

    InsStatus applyBaroCorrection(
        NStateVec& nom_state, 
        ESCovarMtx& covar, 
        const BaroCorrectionData& baro_sample, 
        uint64_t dt
    );

    // utility eskf methods 
    Eigen::Quaternionf Eskf::RotationVectorToQuaternion(const Eigen::Vector3f& rotation_vector);

    void Eskf::CorrectionInjection(const Eigen::Vector<float,16>& es, NStateVec& ns);
};

}