/*
Author      - Calvin Gross
Date        - 9/8/26
Modified    - 9/18/26
Modified    - 9/19/26
Title       - Error State Kalman Filter
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the public methods for interacting with the eskf.
*/

#include "eskf.hpp"


bool Eskf::initEskf(void) {

};

bool Eskf::applyMeasurement(const Measurement& new_measurement) {
    DelayedStateBuffer::MeasurementStatus status = ds_buffer.checkIfDelayed(new_measurement.timestamp);

    if (status == DelayedStateBuffer::MeasurementStatus::on_time) {
        int16_t mea_since_checkpnt = ds_buffer.getMeasurementsSinceCheckpoint();
        
        if (mea_since_checkpnt >= DelayedStateBuffer::MeasurementThreshold) {
            return ds_buffer.appendStateAndMeasurement(new_measurement, this->cur_nominal_state, this->cur_covariance_matrix);
        } else {
            return ds_buffer.appendMeasurement(new_measurement);
        }

    } else if (status == DelayedStateBuffer::MeasurementStatus::out_of_bounds) {
        


    // Here is the actual delayed state handling 
    } else if (status == DelayedStateBuffer::MeasurementStatus::delayed) {

    } else {
        return false;
    }
};

bool Eskf::getCurrentState(NominalState& nominal_state) {
    cur_nominal_state = nominal_state;
    return true;
};
