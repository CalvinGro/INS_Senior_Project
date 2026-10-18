/*
Author      - Calvin Gross
Start Date  - 10/13/26
Title       - Delayed State Filter Recalculator
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header file for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/

#include "eskf.h"
#include "ins_types.h"
#include "delayed_state_fr.h"

namespace ins {

DelayStatus DelayedStateFR::checkIfDelayed(uint64_t timestamp) {

    // check if timestamp is on-time (the newest timestamp)
    if(tail_measurement_ == -1 || MeasurementList[tail_measurement_].measurement.timestamp <= timestamp) {
        return DelayStatus::on_time;
    }

    // check if timestamp is out-of-bounds (past the saved measurements and states)
    uint64_t oldest_timestamp_ = getStateTime(oldest_state_);
    if (oldest_timestamp_ > timestamp) {
        return DelayStatus::out_of_bounds;
    }

    // otherwise the timestamp is somewhere within the saved measurements
    return DelayStatus::delayed;
};


InsStatus DelayedStateFR::insertDelayedMeasurement(const Measurement& new_measurement) {

    // get the first state index to start from
    int16_t cur_state_index = 0;
    InsStatus recalc_status = findRecalcCheckpoint(new_measurement.timestamp, cur_state_index);
    if (recalc_status != InsStatus::ok) return recalc_status;

    FullState cur_state = StateBuffer[cur_state_index].saved_state;

    // find associated measurement
    int16_t par_mea_index;
    int16_t cur_mea_index = StateBuffer[cur_state_index].measurement_index;

    // cur_mea_index and cur_state are the meaurement about to be propagated and the state before it

    // propagate measurements until new_measurement location is found
    while(MeasurementList[cur_mea_index].measurement.timestamp <= new_measurement.timestamp) {
        // return if mea with state found first
        if (
            MeasurementList[cur_mea_index].state_index != -1 && 
            StateBuffer[cur_state_index].measurement_index != cur_mea_index
        ) return InsStatus::bi_search_found_wrong_state;

        // apply current measurement
        InsStatus apply_status = eskf_.applyMeasurement(MeasurementList[cur_mea_index].measurement, cur_state);
        if (apply_status != InsStatus::ok) return apply_status;

        // set parent index
        par_mea_index = cur_mea_index;

        // advance to next measurement
        InsStatus next_status = nextMeasurement(cur_mea_index);
        if (next_status != InsStatus::ok) return next_status;
    }
    // backtrack one index to last measurement already applied, which is before the new_measurement
    cur_mea_index = par_mea_index;

    // insert new measurement into MeasurementList and link it properly
    InsStatus insert_status = insertMeasurementAfter(cur_mea_index, new_measurement);
    if (insert_status != InsStatus::ok) return insert_status;

    // advance to newly inserted measurement
    InsStatus next_status = nextMeasurement(cur_mea_index);
    if (next_status != InsStatus::ok) return next_status;

    // continue propagating along the measurements and updating the attached states
    while(cur_mea_index != -1) { 
        // check if cur mea has an attached state and update it before incorporating the cur mea if so
        if (MeasurementList[cur_mea_index].state_index != -1) {
            StateBuffer[MeasurementList[cur_mea_index].state_index].saved_state = cur_state;
        }
        // apply current measurement
        InsStatus apply_status = eskf_.applyMeasurement(MeasurementList[cur_mea_index].measurement, cur_state);
        if (apply_status != InsStatus::ok) return apply_status;
        // advance to next state (while loop stops at indexed_past_last_measurement)
        InsStatus next_status = nextMeasurement(cur_mea_index);
        if (next_status != InsStatus::ok || next_status != InsStatus::indexed_past_last_measurement) return next_status;
    }

    // once the last measurement is incorporated set current state
    eskf_.cur_state = cur_state;

    return InsStatus::ok;
};
}