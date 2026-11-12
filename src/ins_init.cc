/*
Author      - Calvin Gross
Start Date  - 11/5/26
Title       - Ins Initialization methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the methods to handle each measurement type during
            the initialization phase along with the the init method that finalizes 
            all the starting values for the eskf and ins.
*/

#include "ins_types.h"
#include "eskf.h"
#include "ins.h"


#define INIT_AG_DT ((1.0f / 200.0f)) // in seconds because vgr values use seconds

namespace ins {

InsStatus Ins::init(void) {
    
};

InsStatus Ins::initBaroAccumulate(
    float pressure,
    uint64_t timestamp
) {
    // determine new mean and covariance
    // uses a incremental mean calculation with fixed variance inverse-variance weighting
    eskf.init_state.baro_count += 1;
    if (eskf.init_state.baro_count == 1) {
        eskf.init_state.baro_pres_mean = pressure;
        eskf.init_state.baro_pres_var = eskf.conf_baro.var;
    } else {
        eskf.init_state.baro_pres_mean = 
            eskf.init_state.baro_pres_mean + ((pressure - eskf.init_state.baro_pres_mean) / eskf.init_state.baro_count);
        eskf.init_state.baro_pres_var = eskf.conf_baro.var / eskf.init_state.baro_count;
    }

    if (Ins::readyToStart(timestamp)) {
        return Ins::init();
    }
    return InsStatus::ok;
};    

InsStatus Ins::initMagAccumulate(
    Eigen::Vector3f mag_field,
    uint64_t timestamp
) {
    // determine new mean and covariance vectors
    // uses a incremental mean calculation with fixed variance inverse-variance weighting
    eskf.init_state.mag_count += 1;
    if (eskf.init_state.mag_count == 1) {
        eskf.init_state.mag_field_mean = mag_field;
        eskf.init_state.mag_field_var = eskf.conf_mag.mag_var;
    } else {
        eskf.init_state.mag_field_mean = 
            eskf.init_state.mag_field_mean + ((mag_field - eskf.init_state.mag_field_mean) / eskf.init_state.mag_count);
        eskf.init_state.mag_field_var = eskf.conf_mag.mag_var / eskf.init_state.mag_count;
    }

    if (Ins::readyToStart(timestamp)) {
        return Ins::init();
    }
    return InsStatus::ok;
};

InsStatus Ins::initGnssAccumulate(
    Eigen::Vector3d ecef_pos,
    Eigen::Vector3f gnss_var,
    double lat_rad,
    double long_rad,
    uint64_t timestamp
) {
    eskf.init_state.gnss_count += 1;
    if (eskf.init_state.gnss_count == 1) {
        eskf.init_state.ecef_pos_mean = ecef_pos;
        eskf.init_state.enu_pos_var = gnss_var;
    } else {        
        const Eigen::Vector3f K_gain_enu = 
            eskf.init_state.enu_pos_var.cwiseQuotient(eskf.init_state.enu_pos_var + gnss_var);
        const Eigen::Matrix3d K_gain_ecef = rotateEnuToEskf(K_gain_enu.cast<double>(), lat_rad, long_rad);

        eskf.init_state.enu_pos_var = gnss_var.cwiseProduct(K_gain_enu);
        eskf.init_state.ecef_pos_mean += K_gain_ecef * (ecef_pos - eskf.init_state.ecef_pos_mean);
    }
    if (Ins::readyToStart(timestamp)) {
        return Ins::init();
    }
    return InsStatus::ok;
};

InsStatus Ins::initAccelGyroAccumulate(
        Eigen::Vector3f accel,
        Eigen::Vector3f ang_vel,
        uint64_t timestamp 
) {
    eskf.init_state.ag_count += 1;
    if (eskf.init_state.ag_count == 1) {
        eskf.init_state.accel_mean = accel;
        eskf.init_state.accel_var = eskf.conf_accel.accel_vgr / INIT_AG_DT;
        eskf.init_state.ang_vel_mean = ang_vel;
        eskf.init_state.ang_vel_var = eskf.conf_gyro.ang_vel_vgr / INIT_AG_DT;
    } else {
        // determine change in time
        const uint64_t dt = timestamp - eskf.init_state.prev_ag_timestamp; 
        if (dt == 0) {
            eskf.init_state.ag_count -= 1;
            return InsStatus::duplicate_ag_timestamps_during_init;
        }
        const float dt_sec = static_cast<float>(dt) * 1e-3f;
        
        Ins::accumulate(
            accel, 
            (eskf.conf_accel.accel_vgr / (float)dt_sec),
            eskf.init_state.accel_mean,
            eskf.init_state.accel_var
        );
        Ins::accumulate(
            ang_vel, 
            (eskf.conf_gyro.ang_vel_vgr / (float)dt_sec),
            eskf.init_state.ang_vel_mean,
            eskf.init_state.ang_vel_var
        );
    }
    eskf.init_state.prev_ag_timestamp = timestamp;

    if (Ins::readyToStart(timestamp)) {
        return Ins::init();
    }
    return InsStatus::ok;
};


// Utility Methods Below

bool Ins::readyToStart(uint64_t timestamp) {
    if (timestamp < (eskf.init_state.first_timestamp + 2000)) return false;
    if (eskf.init_state.ag_count < 200) return false;
    if (eskf.init_state.baro_count < 30) return false;
    if (eskf.init_state.mag_count < 30) return false;
    if (eskf.init_state.gnss_count < 5) return false;
    return true;
};

// acculmulation method for measurements with non-fixed variances
void Ins::accumulate(
    const Eigen::Vector3f& measurement,
    const Eigen::Vector3f& measurement_var,
    Eigen::Vector3f& mean,
    Eigen::Vector3f& mean_var
) {
    const Eigen::Vector3f K_gain = mean_var.cwiseQuotient(mean_var + measurement_var);
    mean_var = measurement_var.cwiseProduct(K_gain);
    mean = mean + K_gain.cwiseProduct(measurement - mean);
};

// method to rotate a variance expressed in enu into the ecef frame for a given geolocation
Eigen::Matrix3d Ins::rotateEnuToEskf(
    const Eigen::Vector3d enu_variance,
    const double latitude, // radians
    const double longitude // radians
) {
    const double sin_lat = std::sin(latitude);
    const double cos_lat = std::cos(latitude);
    const double sin_long = std::sin(longitude);
    const double cos_long = std::cos(longitude);

    Eigen::Matrix3d R_enu_to_ecef;
    R_enu_to_ecef <<
        -sin_long, -sin_lat * cos_long, cos_lat * cos_long,
         cos_long, -sin_lat * sin_long, cos_lat * sin_long,
              0.0,             cos_lat,            sin_lat;

    return R_enu_to_ecef * enu_variance.asDiagonal() * R_enu_to_ecef.transpose();
};
};
