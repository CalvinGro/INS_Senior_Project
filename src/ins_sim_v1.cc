/*
Author      - Calvin Gross
Start Date  - 10/23/26
Title       - Ins Simulation version 1
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the interface to apply abstracted measurements of
            simple states without any sensor nuance to the Eskf. It provides a 
            minimal initialization method and state getter.
*/

#include "ins_sim_v1.h"

#include "ins_types.h"
#include "eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <variant>

namespace ins {

InsStatus InsSimV1::initSimV1Ins(void) {
    // don't have to track any initial reference frame relative to 
    // magnetic north, mean sea-level, or ECEF. I assume everything starts at 0.
};

InsStatus InsSimV1::getCurrentState(NStateVec& nominal_state) {
    nominal_state = eskf.cur_state.ns;
    return InsStatus::ok;
};



void InsSimV1::receiveV1AccelGyro(const AGExternalV1Sample& ex_ag_sample) {
    AGInternalSample in_ag_sample {
        Eigen::Vector3f{ex_ag_sample.accel_x, ex_ag_sample.accel_y, ex_ag_sample.accel_z},
        Eigen::Vector3f{ex_ag_sample.ang_vel_x, ex_ag_sample.ang_vel_y, ex_ag_sample.ang_vel_z}
    };
    InternalMeasurement in_ag_mea {in_ag_sample, ex_ag_sample.timestamp};
    processV1Measurement(in_ag_mea);
};

void InsSimV1::receiveV1Baro(const BaroExternalV1Sample& ex_baro_sample) {
    BaroInternalSample in_baro_sample {ex_baro_sample.z_pos};
    InternalMeasurement in_baro_mea {in_baro_sample, ex_baro_sample.timestamp};
    processV1Measurement(in_baro_mea);
};

void InsSimV1::receiveV1Mag(const MagExternalV1Sample& ex_mag_sample) {
    MagInternalSample in_mag_sample {
        Eigen::Vector3f{ex_mag_sample.ang_pos_x, ex_mag_sample.ang_pos_y, ex_mag_sample.ang_pos_z}
    };
    InternalMeasurement in_mag_mea {in_mag_sample, ex_mag_sample.timestamp};
    processV1Measurement(in_mag_mea);
};

void InsSimV1::receiveV1Gnss(const GnssExternalV1Sample& ex_gnss_sample) {
    GnssInternalSample in_gnss_sample {
        Eigen::Vector3f{ex_gnss_sample.pos_x, ex_gnss_sample.pos_y, ex_gnss_sample.pos_z},
        Eigen::Vector3f{ex_gnss_sample.gnss_var_x, ex_gnss_sample.gnss_var_y, ex_gnss_sample.gnss_var_z}
    };
    InternalMeasurement in_gnss_mea {in_gnss_sample, ex_gnss_sample.timestamp};
    processV1Measurement(in_gnss_mea);
};



void InsSimV1::processV1Measurement(const InternalMeasurement& measurement) {
    DelayStatus delay_status = delayed_fr.checkIfDelayed(measurement.timestamp);
    if (delay_status == DelayStatus::delayed) {
        InsStatus insert_mea_status = delayed_fr.insertDelayedMeasurement(measurement);
        if (insert_mea_status != InsStatus::ok) appendError(insert_mea_status);
        
    } else if (delay_status == DelayStatus::on_time) {
        InsStatus apply_status = eskf.applyMeasurement(measurement);
        if (apply_status != InsStatus::ok) {
            appendError(apply_status);
            return;
        }
        
        InsStatus append_status = delayed_fr.appendOnTimeMeasurement(measurement);
        if (append_status != InsStatus::ok) appendError(append_status);
        
    } else {
        appendError(InsStatus::measurement_out_of_fr_range);
    }
};

// private method to handle adding errors
void InsSimV1::appendError(const InsStatus error) {
    if (loe_i < LIST_OF_ERRORS_SIZE) {
        if (loe_i == LIST_OF_ERRORS_SIZE-1) {
            list_of_errors[loe_i] = InsStatus::loe_overflow;
        } else {
            list_of_errors[loe_i] = error;
        }
    }
}
}