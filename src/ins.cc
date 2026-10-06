/*
Author      - Calvin Gross
Date        - 9/8/26
Modified    - 9/18/26
Modified    - 9/19/26
Title       - Error State Kalman Filter
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the public methods for interacting with the eskf.
*/

#include "delayed_state_buffer.h"
#include "eskf.h" 
#include "ins_types.h"

#include <stdbool.h>
#include <variant>
#include <stdint.h>

namespace ins {

bool Eskf::initEskf(void) {

};

bool Ins::applyMeasurement(const Measurement& new_measurement) {
    DelayedStateBuffer::DelayStatus status = ds_buffer.checkIfDelayed(new_measurement.timestamp);

    // Standard, non-delayed measurement handling.
    if (status == DelayedStateBuffer::DelayStatus::on_time) {
        int16_t mea_since_checkpnt = ds_buffer.getMeasurementsSinceCheckpoint();
        bool append_status = true; 

        // save state only if threshold has been reached and the previous measurement is a
        // accel/gyro measurement, so that the saved previous timestamp is relevant.
        if (mea_since_checkpnt >= DelayedStateBuffer::MeasurementThreshold &&
            std::holds_alternative<AccelGyroUpdateData>(new_measurement.data)) {
            append_status =  ds_buffer.appendStateAndMeasurement(new_measurement, this->cur_nominal_state, this->cur_covariance_matrix);
        } else {
            append_status = ds_buffer.appendMeasurement(new_measurement);
        }
        if (!append_status) return append_status;

        if (new_measurement.timestamp < cur_ag_timestamp) return false;
            uint64_t dt = new_measurement.timestamp - cur_ag_timestamp;

        // apply to current state and covariance.
        bool apply_status = std::visit([this, dt](const auto& sample) -> bool {
            using T = std::decay_t<decltype(sample)>;

            // constexpr only includes the branch with the correct type after compilation
            // for each types generated lambda function.
            if constexpr (std::is_same_v<T, AccelGyroUpdateData>) {
                return AccelGyroPredict(this->cur_nominal_state, this->cur_covariance_matrix, sample, dt);
            } else if constexpr (std::is_same_v<T, MagCorrectionData>) {
                return applyMagCorrection(this->cur_nominal_state, this->cur_covariance_matrix, sample, dt);
            } else if constexpr (std::is_same_v<T, GnssCorrectionData>) {
                return applyGnssCorrection(this->cur_nominal_state, this->cur_covariance_matrix, sample, dt);
            } else if constexpr (std::is_same_v<T, BaroCorrectionData>) {
                return applyBaroCorrection(this->cur_nominal_state, this->cur_covariance_matrix, sample, dt);
            }
        }, new_measurement.data);
        if (!apply_status) return apply_status;
    
        // update timestamp if it is a accel/gyro measurement.
        if (std::holds_alternative<AccelGyroUpdateData>(new_measurement.data)) {
            cur_ag_timestamp = new_measurement.timestamp;
        }


    // Handle the new measurement having a timestamp before any saved states.
    } else if (status == DelayedStateBuffer::DelayStatus::out_of_bounds) {
        if (oob_log_i >= OutBoundsLogSize) return true;

        // update out of bounds log if it is not full yet
        out_of_bounds_log[oob_log_i] = new_measurement;
        oob_log_i++;


    // Here is the actual delayed state handling.
    } else if (status == DelayedStateBuffer::DelayStatus::delayed) {
        int16_t start_state_i = ds_buffer.getStartState(new_measurement.timestamp);
        if (start_state_i == -1) return false;

        int16_t measurement_i = ds_buffer.StateBuffer[start_state_i].measurement_index;
        int16_t prev_mea_i = -1;

        uint64_t prev_ag_timestamp = ds_buffer.StateBuffer[start_state_i].prev_timestamp;
        NominalState tracked_state = ds_buffer.StateBuffer[start_state_i].nominal_state;
        CovarianceMatrix tracked_covar = ds_buffer.StateBuffer[start_state_i].covar_matrix;

        bool applied_new = false;

        // loop through all measurements, add the new measurement, and modify the saved and current states
        while (measurement_i != -1) {

            // apply new measurement once spot found
            if (!applied_new && new_measurement.timestamp < ds_buffer.MeasurementList[measurement_i].measurement.timestamp) {
                
                // current measurement should advance past the first measurement (the one linked to the state)
                // before the new measurement insertion, so prev_mea should not be -1.
                if (prev_mea_i != -1) {
                    bool insert_status = ds_buffer.insertMeasurementAfter(prev_mea_i, new_measurement);
                    if (!insert_status) return insert_status;

                    // now that the current measurement has been updated to the new measurement, use that one
                    measurement_i = ds_buffer.MeasurementList[prev_mea_i].next_measurement_index;

                    applied_new = true;
                } else {
                    return false;
                }
            }

            // first, check if there is a state attached and if so, 
            // update it with the states before the new_measurement.
            if (ds_buffer.MeasurementList[measurement_i].state_index != -1 && 
                ds_buffer.MeasurementList[measurement_i].state_index != start_state_i) {
                ds_buffer.StateBuffer[ds_buffer.MeasurementList[measurement_i].state_index].nominal_state = tracked_state;
                ds_buffer.StateBuffer[ds_buffer.MeasurementList[measurement_i].state_index].covar_matrix = tracked_covar;
                ds_buffer.StateBuffer[ds_buffer.MeasurementList[measurement_i].state_index].prev_timestamp = prev_ag_timestamp;
            }
            
            // calculate the change in time (dt) since the last accel/gyro reading.
            if (ds_buffer.MeasurementList[measurement_i].measurement.timestamp < prev_ag_timestamp) return false;
            uint64_t dt = ds_buffer.MeasurementList[measurement_i].measurement.timestamp - prev_ag_timestamp;
            
            // next, apply the current measurement to the state and covariance 
            // using the correct function based off the type of the data.
            bool apply_status = std::visit([this, &tracked_state, &tracked_covar, dt](const auto& sample) -> bool {
                using T = std::decay_t<decltype(sample)>;

                // constexpr only includes the branch with the correct type after compilation
                // for each types generated lambda function.
                if constexpr (std::is_same_v<T, AccelGyroUpdateData>) {
                    return AccelGyroPredict(tracked_state, tracked_covar, sample, dt);
                } else if constexpr (std::is_same_v<T, MagCorrectionData>) {
                    return applyMagCorrection(tracked_state, tracked_covar, sample, dt);
                } else if constexpr (std::is_same_v<T, GnssCorrectionData>) {
                    return applyGnssCorrection(tracked_state, tracked_covar, sample, dt);
                } else if constexpr (std::is_same_v<T, BaroCorrectionData>) {
                    return applyBaroCorrection(tracked_state, tracked_covar, sample, dt);
                }
            }, ds_buffer.MeasurementList[measurement_i].measurement.data);
            if (!apply_status) return apply_status;

            // set the previous accel/gyro timestamp if the current measurement is accel/gyro.
            if (std::holds_alternative<AccelGyroUpdateData>(ds_buffer.MeasurementList[measurement_i].measurement.data)) {
                prev_ag_timestamp = ds_buffer.MeasurementList[measurement_i].measurement.timestamp;
            }

            // progress to the next measurement
            prev_mea_i = measurement_i;
            measurement_i = ds_buffer.MeasurementList[measurement_i].next_measurement_index;
        }

        // verify new_measurement was applied
        if (!applied_new) return false;

        // after the final measurement save the state and covariance
        cur_nominal_state = tracked_state;
        cur_covariance_matrix = tracked_covar;
        cur_ag_timestamp = prev_ag_timestamp;

    } else {
        return false;
    }

    return true;
};

bool Eskf::getCurrentState(NominalState& nominal_state) {
    nominal_state = cur_nominal_state;
    return true;
};

}