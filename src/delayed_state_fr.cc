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


InsStatus DelayedStateFR::insertMeasurement(const Measurement& new_measurement) {

    // get the first state index to start from
    int16_t cur_state_index = 0;
    InsStatus recalc_status = findRecalcCheckpoint(new_measurement.timestamp, cur_state_index);
    if (recalc_status != InsStatus::ok) return recalc_status;

    // find associated measurement
    int16_t cur_mea_index = StateBuffer[cur_state_index].measurement_index;

    // propagate measurements

}
}