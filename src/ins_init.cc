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

#include <limits>
#include <cmath>

#define INIT_AG_DT ((1.0f / 200.0f)) // in seconds because vgr values use seconds

namespace ins {

InsStatus Ins::init(uint64_t last_timestamp) {

    // gyro config tranlations and settings
    Eigen::Matrix3d gyro_transform = eskf.conf_gyro.sensor_to_body.cast<double>() * eskf.conf_gyro.correction.cast<double>();
    eskf.settings.gyro_correction = gyro_transform.cast<float>();
    eskf.settings.gyro_offset = eskf.conf_gyro.offset;

    // accel config tranlations and settings
    const Eigen::Matrix3d accel_transform = eskf.conf_accel.sensor_to_body.cast<double>() * eskf.conf_accel.correction.cast<double>();
    eskf.settings.accel_correction = accel_transform.cast<float>();
    eskf.settings.accel_offset = eskf.conf_accel.offset;
    const Eigen::Vector3d accel_mean_body = 
        accel_transform * (eskf.init_state.accel_mean.cast<double>() - eskf.conf_accel.offset.cast<double>());

    // mag config tranlations and settings
    const Eigen::Matrix3d mag_transform = eskf.conf_mag.sensor_to_body.cast<double>() * eskf.conf_mag.correction.cast<double>();
    eskf.settings.mag_correction = mag_transform.cast<float>();
    eskf.settings.mag_offset = eskf.conf_mag.offset;
    eskf.settings.mag_field_enu = eskf.conf_mag.mag_field_enu;
    const Eigen::Vector3d mag_mean_body = 
        mag_transform * (eskf.init_state.mag_field_mean.cast<double>() - eskf.conf_mag.offset.cast<double>());

    // use magnetometer and accelerometer to perform 2 axis triad estimate of the orientation.
    Eigen::Matrix<double,6,6> orient_input_covar = Eigen::Matrix<double, 6, 6>::Zero();
    orient_input_covar.block<3,3>(0,0) = accel_transform * eskf.init_state.accel_var.asDiagonal() * accel_transform.transpose();
    orient_input_covar.block<3,3>(3,3) = mag_transform * eskf.init_state.mag_field_var.asDiagonal() * mag_transform.transpose(); 
    Eigen::Matrix3d R0;
    Eigen::Matrix<double,3,6> J = buildOrientationJacobian(accel_mean_body, mag_mean_body, R0);    
    const Eigen::Matrix3f orient_covar = (J * orient_input_covar * J.transpose()).cast<float>(); // enu orientation error
    const Eigen::Quaternionf orient_mean = Eigen::Quaterniond(R0).normalized().cast<float>(); // enu orientation mean

    // calculate the starting position in ecef and use it as the reference frame
    eskf.settings.origin_ecef = eskf.init_state.ecef_pos_mean;

    // calculate the barometer altitude offset and variance in meters
    eskf.settings.baro_offset = translatePressure(eskf.init_state.baro_pres_mean);
    
    // constants given by www.weather.gov pressureAltitude data
    constexpr double A  = 44307.69396;
    constexpr double k  = 0.190284;
    constexpr double p0 = 101325.0;

    // calculate the derivative of pressure to altitude function
    const double dh_dp = -(A * k / p0) * std::pow(eskf.init_state.baro_pres_mean / p0, k - 1.0);
    const double baro_pos_var = dh_dp * dh_dp * eskf.init_state.baro_pres_var; // m²
    
    // declare the starting state
    eskf.cur_state.last_pred = eskf.init_state.last_ag;
    eskf.cur_state.last_timestamp = last_timestamp;
    
    eskf.cur_state.ns.position = {0,0,0}; // initial ecef offset is used to initalize the ref frame at zero
    eskf.cur_state.ns.velocity = {0,0,0}; // assuming stationary startup means no initial velocity
    eskf.cur_state.ns.angular_position = orient_mean;
    eskf.cur_state.ns.accel_bias = {0,0,0}; // the accel bias is unobservable due to accel's use in triad
    eskf.cur_state.ns.baro_bias = 0.0f; // initial bias is used as altitude bias, but the variance is still set
    eskf.cur_state.ns.gyro_bias = eskf.settings.gyro_correction * 
        (eskf.init_state.ang_vel_mean - eskf.settings.gyro_offset).asDiagonal() * eskf.settings.gyro_correction.transpose();
    
    eskf.cur_state.escm = ESCovarMtx::Identity();
    eskf.cur_state.escm.block<3,3>(0,0) = eskf.init_state.enu_pos_var;
    eskf.cur_state.escm.block<3,3>(3,3) = Eigen::Matrix3f::Identity() * (0.01f * 0.01f); // add a small error despite assumption of stationary init
    eskf.cur_state.escm.block<3,3>(6,6) = orient_covar;
    eskf.cur_state.escm.block<3,3>(9,9) =  eskf.settings.accel_correction * 
        eskf.conf_accel.initial_residual_bias_var.asDiagonal() * eskf.settings.accel_correction.transpose(); // accel bias var
    eskf.cur_state.escm.block<3,3>(12,12) = eskf.settings.gyro_correction * 
        eskf.init_state.ang_vel_var.asDiagonal() * eskf.settings.gyro_correction.transpose(); // gyro bias var
    eskf.cur_state.escm(15,15) = baro_pos_var; // baro bias var

    // initialize the delayed state filter with a first measurement

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
        return Ins::init(timestamp);
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
        return Ins::init(timestamp);
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
        eskf.settings.ecef_to_enu = calcRotationEnuToEskf(lat_rad, long_rad);
    } else {        
        const Eigen::Vector3f K_gain_enu = 
            eskf.init_state.enu_pos_var.cwiseQuotient(eskf.init_state.enu_pos_var + gnss_var);
        Eigen::Matrix3d K_gain_ecef = 
            eskf.settings.ecef_to_enu * K_gain_enu.cast<double>() * eskf.settings.ecef_to_enu.transpose();
        eskf.init_state.enu_pos_var = gnss_var.cwiseProduct(K_gain_enu);
        eskf.init_state.ecef_pos_mean += K_gain_ecef * (ecef_pos - eskf.init_state.ecef_pos_mean);
    }
    if (Ins::readyToStart(timestamp)) {
        return Ins::init(timestamp);
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
    eskf.init_state.last_ag = { accel, ang_vel };

    if (Ins::readyToStart(timestamp)) {
        return Ins::init(timestamp);
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
Eigen::Matrix3d Ins::calcRotationEnuToEskf(
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

    return R_enu_to_ecef;
};


// Utility methods for 2 axis orientation initial estimate calculation using TRIAD
Eigen::Matrix<double,3,6> Ins::buildOrientationJacobian(
    const Eigen::Vector3d& est_accel,
    const Eigen::Vector3d& est_mag,
    Eigen::Matrix3d& rotation0
) {
    const double step = std::cbrt(std::numeric_limits<double>::epsilon());

    Eigen::Matrix<double,3,6> J;

    const Eigen::Vector3d grav = {0, 0, 9.80655};
    const Eigen::Vector3d mf = eskf.conf_mag.mag_field_enu.cast<double>();

    // Construct the actual rotation
    rotation0 = triad2AxisCalc(est_accel, est_mag, grav, mf);
    const Eigen::Matrix3d R0T = rotation0.transpose();

    const Eigen::Vector3d x_step = {step, 0, 0};
    const Eigen::Vector3d y_step = {0, step, 0};
    const Eigen::Vector3d z_step = {0, 0, step};

    // accel gravity axis
    {
        const Eigen::Vector3d Rax_p = RotMaToVec(triad2AxisCalc(est_accel + x_step, est_mag, grav, mf) * R0T);
        const Eigen::Vector3d Rax_n = RotMaToVec(triad2AxisCalc(est_accel - x_step, est_mag, grav, mf) * R0T);
        J.col(0) = (Rax_p - Rax_n) / (2 * step);
    }
    {
        const Eigen::Vector3d Ray_p = RotMaToVec(triad2AxisCalc(est_accel + y_step, est_mag, grav, mf) * R0T);
        const Eigen::Vector3d Ray_n = RotMaToVec(triad2AxisCalc(est_accel - y_step, est_mag, grav, mf) * R0T);
        J.col(1) = (Ray_p - Ray_n) / (2 * step);
    }
    {
        const Eigen::Vector3d Raz_p = RotMaToVec(triad2AxisCalc(est_accel + z_step, est_mag, grav, mf) * R0T);
        const Eigen::Vector3d Raz_n = RotMaToVec(triad2AxisCalc(est_accel - z_step, est_mag, grav, mf) * R0T);
        J.col(2) = (Raz_p - Raz_n) / (2 * step);
    }

    // magnetic field axis
    {
        const Eigen::Vector3d Rmx_p = RotMaToVec(triad2AxisCalc(est_accel, est_mag + x_step, grav, mf) * R0T);
        const Eigen::Vector3d Rmx_n = RotMaToVec(triad2AxisCalc(est_accel, est_mag - x_step, grav, mf) * R0T);
        J.col(3) = (Rmx_p - Rmx_n) / (2 * step);
    }
    {
        const Eigen::Vector3d Rmy_p = RotMaToVec(triad2AxisCalc(est_accel, est_mag + y_step, grav, mf) * R0T);
        const Eigen::Vector3d Rmy_n = RotMaToVec(triad2AxisCalc(est_accel, est_mag - y_step, grav, mf) * R0T);
        J.col(4) = (Rmy_p - Rmy_n) / (2 * step);
    }
    {
        const Eigen::Vector3d Rmz_p = RotMaToVec(triad2AxisCalc(est_accel, est_mag + z_step, grav, mf) * R0T);
        const Eigen::Vector3d Rmz_n = RotMaToVec(triad2AxisCalc(est_accel, est_mag - z_step, grav, mf) * R0T);
        J.col(5) = (Rmz_p - Rmz_n) / (2 * step);
    }
    return J;
};


Eigen::Matrix3d Ins::triad2AxisCalc(
    const Eigen::Vector3d& est_axis_1,
    const Eigen::Vector3d& est_axis_2,
    const Eigen::Vector3d& true_axis_1,
    const Eigen::Vector3d& true_axis_2
) {
    Eigen::Matrix3d est_axes;
    est_axes.col(0) = est_axis_1.normalized(); // orthoginal, unit length estimate axis 1
    est_axes.col(1) = est_axes.col(0).cross(est_axis_2).normalized(); // orthoginal, unit length estimate axis 2
    est_axes.col(2) = est_axes.col(0).cross(est_axes.col(1)); // orthoginal, unit length estimate axis 3

    Eigen::Matrix3d true_axes;
    true_axes.col(0) = true_axis_1.normalized();
    true_axes.col(1) = true_axes.col(0).cross(true_axis_2).normalized();
    true_axes.col(2) = true_axes.col(0).cross(true_axes.col(1));

    return true_axes * est_axes.inverse().eval();
};


Eigen::Vector3d Ins::RotMaToVec(
    const Eigen::Matrix3d& rot_matrix
) {
    const Eigen::AngleAxisd axis_angle(rot_matrix);
    return axis_angle.angle() * axis_angle.axis();
};

};
