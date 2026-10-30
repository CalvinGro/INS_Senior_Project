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
public:

    // the most recent state info
    FullState cur_state;

    // generic method to apply any measuremnt type
    InsStatus applyMeasurement(const InternalMeasurement& new_measurement);
    
    InsStatus applyMeasurement(const InternalMeasurement& new_measurement, FullState& prev_state);


    // constants that are preconfigured or set during init before initializing the Eskf
    MagConfiguration conf_mag;
    AccelConfiguration conf_accel;
    GyroConfiguration conf_gyro;
    GnssConfiguration conf_gnss;
    BaroConfiguration conf_baro;
    
private:
    inline static const Eigen::Vector3f gravity = {0, 0, -9.80655};


    // methods defined in eskf_prediction.cpp

    InsStatus applyAccelGyroPredict(
        uint64_t dt,
        const AGInternalSample& ag_sample, 
        FullState& prev_state
    );

    // overloaded - Used when updating the current state
    InsStatus applyAccelGyroPredict(
        uint64_t dt,
        const AGInternalSample& ag_sample
    );

    // overloaded - Used when propagating to a given timestamp before correction
    InsStatus applyAccelGyroPredict(uint64_t dt);


    // methods defined in eskf_correction.cpp

    InsStatus applyMagCorrection(
        uint64_t dt,
        const MagInternalSample& mag_sample,
        FullState& prev_state
    );


    InsStatus applyGnssCorrection(
        uint64_t dt,
        const GnssInternalSample& gnss_sample,
        FullState& prev_state
    );

    // overloaded - Used when correcting the current state with a gnss sample
    InsStatus applyGnssCorrection(
        uint64_t dt,
        const GnssInternalSample& gnss_sample
    );


    InsStatus applyBaroCorrection(
        uint64_t dt,
        const BaroInternalSample& baro_sample,
        FullState& prev_state
    );

    // overloaded - Used when correcting the current state with a barometer sample
    InsStatus applyBaroCorrection(
        uint64_t dt,
        const BaroInternalSample& baro_sample
    );

    // utility eskf methods 
    Eigen::Quaternionf rotationVectorToQuaternion(const Eigen::Vector3f& rotation_vector);

    void correctionInjection(const Eigen::Vector<float,16>& es, NStateVec& ns);
};

}