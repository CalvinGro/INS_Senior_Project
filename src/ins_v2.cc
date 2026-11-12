/*
Author      - Calvin Gross
Start Date  - 11/5/26
Title       - Ins Interface 
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the interface to internally process actual measurements
            from sensors and apply them to the filter to determine actual states. This is
            used for both version 2 simulation and filtering gathered readings.
*/

#include "ins_types.h"
#include "eskf.h"
#include "ins.h"

#include <numbers>
#include <cmath>

namespace ins {
    InsStatus Ins::applyAccelGyroMeasurement(
        const float x_accel,
        const float y_accel,
        const float z_accel,
        const float x_ang_vel,
        const float y_ang_vel,
        const float z_ang_vel,
        const uint64_t timestamp
    ) {
        Eigen::Vector3f accel = {x_accel, y_accel, z_accel};
        Eigen::Vector3f ang_vel = {x_ang_vel, y_ang_vel, z_ang_vel};

        // Check if in initialization Phase
        if (eskf.init_state.init_phase == InsPhase::uninitialized) {
            eskf.init_state.first_timestamp = timestamp;
            eskf.init_state.init_phase = InsPhase::initializing;
        }
        if (eskf.init_state.init_phase == InsPhase::initializing) {
            return Ins::initAccelGyroAccumulate(
                accel,
                ang_vel,
                timestamp
            );
        }

        AGInternalSample ag_sample = {
            (eskf.conf_accel.sensor_to_body * eskf.conf_accel.correction) * (accel - eskf.conf_accel.offset),
            (eskf.conf_gyro.sensor_to_body * eskf.conf_gyro.correction) * (ang_vel - eskf.conf_gyro.offset)
        };

        InternalMeasurement mea = {
            ag_sample,
            timestamp
        };
        return eskf.applyMeasurement(mea);
    };


    InsStatus Ins::applyBaroMeasurment(
        const float pressure,
        const uint64_t timestamp
    ) {
        if (!std::isfinite(pressure) || pressure <= 0.0f) {
            return InsStatus::invalid_pressure_measurement;
        }

        // Check if in initialization Phase
        if (eskf.init_state.init_phase == InsPhase::uninitialized) {
            eskf.init_state.first_timestamp = timestamp;
            eskf.init_state.init_phase = InsPhase::initializing;
        }
        if (eskf.init_state.init_phase == InsPhase::initializing) {
            return Ins::initBaroAccumulate(pressure, timestamp);
        }

        float altitude = translatePressure(pressure);
        float z_pos = altitude - eskf.init_values.baro_offset;
        BaroInternalSample baro_sample = { z_pos };
        InternalMeasurement mea = {
            baro_sample,
            timestamp
        };
        return eskf.applyMeasurement(mea);
    };


    InsStatus Ins::applyMagMeasurement(
        const float x_mag_flux,
        const float y_mag_flux,
        const float z_mag_flux,
        const uint64_t timestamp
    ) {
        if (
            !std::isfinite(x_mag_flux) || !std::isfinite(y_mag_flux) || !std::isfinite(z_mag_flux)) {
            return InsStatus::invalid_mag_field_measurement;
        }

        Eigen::Vector3f mag_field = {x_mag_flux, y_mag_flux, z_mag_flux};

        // Check if in initialization Phase
        if (eskf.init_state.init_phase == InsPhase::uninitialized) {
            eskf.init_state.first_timestamp = timestamp;
            eskf.init_state.init_phase = InsPhase::initializing;
        }
        if (eskf.init_state.init_phase == InsPhase::initializing) {
            return Ins::initMagAccumulate(
                mag_field,
                timestamp
            );
        }

        MagInternalSample mag_sample = { 
            (eskf.conf_mag.sensor_to_body * eskf.conf_mag.correction) * (mag_field - eskf.conf_mag.offset) 
        };

        InternalMeasurement mea = {
            mag_sample,
            timestamp
        };
        return eskf.applyMeasurement(mea);
    };
    

    InsStatus Ins::applyGnssMeasurement(
        const double latitude, // degrees
        const double longitude, // degrees
        const float altitude,
        const float hAcc,
        const float vAcc,
        const uint64_t timestamp
    ) {
        if (!std::isfinite(latitude) || !std::isfinite(longitude) || !std::isfinite(altitude)) {
            return InsStatus::invalid_gnss_position_measurement;
        }

        // translate lat and long from degrees to radians
        constexpr double deg_to_rad = std::numbers::pi / 180.0;
        const double lat_rad = latitude * deg_to_rad;
        const double long_rad = longitude * deg_to_rad;

        Eigen::Vector3f gnss_var = {hAcc, hAcc, vAcc};
        if (!std::isfinite(hAcc) || hAcc <= 0.0f) {
            gnss_var[0] = eskf.conf_gnss.default_var[0];
            gnss_var[1] = eskf.conf_gnss.default_var[1];
        }
        if (!std::isfinite(vAcc) || vAcc <= 0.0f) {
            gnss_var[2] = eskf.conf_gnss.default_var[2];
        }

        const Eigen::Vector3d ecef_pos = geodeticToEcef(lat_rad, long_rad, altitude);

        // Check if in initialization Phase
        if (eskf.init_state.init_phase == InsPhase::uninitialized) {
            eskf.init_state.first_timestamp = timestamp;
            eskf.init_state.init_phase = InsPhase::initializing;
        }
        if (eskf.init_state.init_phase == InsPhase::initializing) {
            return Ins::initGnssAccumulate(
                ecef_pos,
                gnss_var,
                lat_rad,
                long_rad,
                timestamp
            );
        }

        const Eigen::Vector3f enu_pos = (eskf.init_values.ecef_to_enu * (ecef_pos - eskf.init_values.origin_ecef)).cast<float>();

        GnssInternalSample gnss_sample = {
            enu_pos,
            gnss_var
        };
        InternalMeasurement mea = {
            gnss_sample,
            timestamp
        };
        return eskf.applyMeasurement(mea);
    };


    float Ins::translatePressure(const float pressure) {
        return 44307.69396f * (1.0f - std::pow((pressure / 101325.0f), 0.190284f));
    };


    Eigen::Vector3d Ins::geodeticToEcef(
        const double lat_rad, 
        const double long_rad, 
        const float altitude
    ) {

        constexpr double a = 6378137.0;           // equatorial radius in meters
        constexpr double f = 1.0 / 298.257223563; // flatting ratio of the earth
        constexpr double e2 = f * (2.0 - f);      // earth's eccentricity squared
        
        const double sin_lat = std::sin(lat_rad);
        const double cos_lat = std::cos(lat_rad);
        const double sin_long = std::sin(long_rad);
        const double cos_long = std::cos(long_rad);

        const double N = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);  // radius of curvature in the prime vertical

        // return ECEF position
        return Eigen::Vector3d{
            (N + altitude) * cos_lat * cos_long,
            (N + altitude) * cos_lat * sin_long,
            (N * (1.0 - e2) + altitude) * sin_lat
        };
    };
};