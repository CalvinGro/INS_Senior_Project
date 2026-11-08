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


namespace ins {
    InsStatus init(void) {

    };

    InsStatus Ins::applyAccelGyroMeasurement(
        float x_accel,
        float y_accel,
        float z_accel,
        float x_ang_vel,
        float y_ang_vel,
        float z_ang_vel,
        uint64_t timestamp
    ) {
        Eigen::Vector3f accel = {x_accel, y_accel, z_accel};
        Eigen::Vector3f ang_vel = {x_ang_vel, y_ang_vel, z_ang_vel};
        Eigen::Matrix3f accel_transformation = eskf.conf_accel.correction * eskf.conf_accel.sensor_to_body;
        Eigen::Matrix3f gyro_transformation = eskf.conf_gyro.correction * eskf.conf_gyro.sensor_to_body;

        AGInternalSample ag_sample = {
            accel_transformation * accel * accel_transformation.transpose(),
            gyro_transformation * ang_vel * gyro_transformation.transpose()
        };

        InternalMeasurement mea = {
            ag_sample,
            timestamp
        };
        eskf.applyMeasurement(mea);
    };

    void Ins::applyBaroMeasurment(
        float pressure,
        uint64_t timestamp
    ) {
        float z_pos = Ins::translatePressure(pressure) - eskf.init_values.baro_offset;
        BaroInternalSample baro_sample = { z_pos };
        InternalMeasurement mea = {
            baro_sample,
            timestamp
        };
        eskf.applyMeasurement(mea);
    };

    InsStatus Ins::applyMagMeasurement(
        float x_mag_flux,
        float y_mag_flux,
        float z_mag_flux,
        uint64_t timestamp
    ) {

    };

    InsStatus Ins::applyGnssMeasurement(
        double lat,
        double long,
        double alt,
        float hAcc,
        float vAcc,
        uint64_t timestamp
    ) {

    };

    float Ins::translatePressure(const float pressure) {
        return 44307.69396f * (1.0f - std::pow((pressure / 101325.0f), 0.190284f));
    };
};