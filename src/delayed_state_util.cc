/*
Author      - Calvin Gross
Start Date  - 10/13/26
Title       - Delayed State Filter Utility Methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header file for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/

#include "ins_types.h"
#include "delayed_state_fr.h"

namespace ins {


// Free-List Methods
InsStatus DelayedStateFR::acquireMeasurementIndex(int16_t acquired_index) {
    if (free_list_index_ == -1) return InsStatus::all_nodes_full;
        if (acquired_index < 0 || acquired_index >= sample_capacity_) {
        return InsStatus::attempted_to_acquire_invalid_index;
    }
    acquired_index = MeasurementFreeList[free_list_index_];
    free_list_index_--;

    return InsStatus::ok;
};

InsStatus DelayedStateFR::releaseMeasurementIndex(int16_t released_index) {
    if (released_index < 0 || released_index >= sample_capacity_) {
        return InsStatus::attempted_to_release_invalid_index;
    }
    free_list_index_++;
    MeasurementFreeList[free_list_index_] = released_index;

    return InsStatus::ok;
};


// Additional Utility Methods
InsStatus DelayedStateFR::findRecalcCheckpoint(const uint64_t timestamp, int16_t found_state_index) {

    // lambda function for transforming bi-search index into circular buffer index
    std::function<int16_t(int16_t)> to_cb_index = [=](int16_t i) {
        return (i + oldest_state_) % state_capacity_;
    };

    // binary search for starting state 
    int16_t i = 0;
    int16_t j = recorded_states_ - 1;
    if (j == -1) {
        return InsStatus::recorded_states_uninit;
    };

    while((i + 1) < j) {
        int16_t mid = i + (j - i) / 2;
        uint64_t mid_time = getStateTime(to_cb_index(mid));
        
        if (timestamp < mid_time) {
            j = mid;
        } else if (timestamp > mid_time) {
            i = mid;
        } else {
            found_state_index = to_cb_index(found_state_index);
            return InsStatus::ok;
        }
    };

    // return the state found
    if (timestamp >= getStateTime(to_cb_index(j))) {
        found_state_index = to_cb_index(found_state_index);
        return InsStatus::ok;
    } else if (timestamp >= getStateTime(to_cb_index(i))) {
        found_state_index = to_cb_index(found_state_index);
        return InsStatus::ok;
    } else {
        return InsStatus::bi_search_failed_in_findRecalcCheckpoint;
    }
};


uint64_t DelayedStateFR::getStateTime(int16_t state_index) {
    return MeasurementList[StateBuffer[state_index].measurement_index].measurement.timestamp;
};


}