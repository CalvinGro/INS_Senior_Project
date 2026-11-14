/*
Author      - Calvin Gross
Start Date  - 11/5/26
Title       - Ins Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines...
*/


#pragma once

#include "delayed_state_fr.h"
#include "eskf.h" 
#include "ins_types.h"

#include <variant>
#include <cmath>
#include <stdint.h>

namespace ins {


class Ins
{

    Eskf eskf = Eskf();
    DelayedStateFR delayed_fr = DelayedStateFR(eskf);

public:
    Ins(
        AccelConfiguration* accel_config, 
        GyroConfiguration* gyro_config, 
        MagConfiguration* mag_config, 
        BaroConfiguration* baro_config, 
        GnssConfiguration* gnss_config
    ) {
        eskf.conf_accel = *accel_config;
        eskf.conf_gyro = *gyro_config;
        eskf.conf_baro = *baro_config;
        eskf.conf_gnss = *gnss_config;
        eskf.conf_mag = *mag_config;
    };

    InsStatus init(uint64_t last_timestamp) {};

    InsStatus initAccelGyroAccumulate(
        Eigen::Vector3f accel,
        Eigen::Vector3f ang_vel,
        uint64_t timestamp 
    ) {};

    InsStatus applyAccelGyroMeasurement(
        float x_accel,
        float y_accel,
        float z_accel,
        float x_orient,
        float y_orient,
        float z_orient,
        uint64_t timestamp  
    ) {};

    InsStatus initBaroAccumulate(
        float pressure,
        uint64_t timestamp
    ) {};

    InsStatus applyBaroMeasurment(
        float pressure,
        uint64_t timestamp
    ) {};

    InsStatus initMagAccumulate(
        Eigen::Vector3f mag_field,
        uint64_t timestamp
    ) {};

    InsStatus applyMagMeasurement(
        float x_mag_flux,
        float y_mag_flux,
        float z_mag_flux,
        uint64_t timestamp
    ) {};

    InsStatus initGnssAccumulate(
        Eigen::Vector3d ecef_pos,
        Eigen::Vector3f gnss_var,
        double lat_rad,
        double long_rad,
        uint64_t timestamp
    ) {};

    InsStatus applyGnssMeasurement(
        double latitude,
        double longitude,
        float altitude,
        float hAcc,
        float vAcc,
        uint64_t timestamp
    ) {};

private:
    InsStatus safeApplyMeasurement(InternalMeasurement& mea) {};

    bool readyToStart(uint64_t timestamp) {};
    
    void accumulate(
        const Eigen::Vector3f& measurement,
        const Eigen::Vector3f& measurement_var,
        Eigen::Vector3f& mean,
        Eigen::Vector3f& mean_var
    ) {};

    float translatePressure(const float pressure) {};

    Eigen::Vector3d geodeticToEcef(
        double latitude, 
        double longitude,
        float altitude
    ) {};

    Eigen::Matrix3d calcRotationEnuToEskf(
        double latitude, // radians
        double longitude // radians
    ) {};

    // initialization utilities
    Eigen::Matrix<double,3,6> buildOrientationJacobian(
        const Eigen::Vector3d& accel,
        const Eigen::Vector3d& mag,
        Eigen::Matrix3d& rotation0
    ) {};

    Eigen::Matrix3d triad2AxisCalc(
        const Eigen::Vector3d& est_axis_1,
        const Eigen::Vector3d& est_axis_2,
        const Eigen::Vector3d& true_axis_1,
        const Eigen::Vector3d& true_axis_2
    ) {};

    Eigen::Vector3d RotMaToVec(
        const Eigen::Matrix3d& rot_matrix
    ) {};


};
};
