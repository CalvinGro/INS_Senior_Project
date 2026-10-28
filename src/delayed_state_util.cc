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
    if (acquired_index < 0 || acquired_index >= sample_capacity_) {
        return InsStatus::attempted_to_acquire_invalid_index;
    }
    // check if measurement linked list is full and clear space if so
    if (recorded_measurements_ == sample_capacity_) {
        InsStatus rm_last = removeLastState();
        if (rm_last != InsStatus::ok) return rm_last;
    }
    // varify it is not still full
    if (recorded_measurements_ == sample_capacity_) return InsStatus::all_nodes_full;

    acquired_index = MeasurementFreeList[free_list_index_];
    free_list_index_--;
    recorded_measurements_++;

    return InsStatus::ok;
};


InsStatus DelayedStateFR::releaseMeasurementIndex(int16_t released_index) {
    if (released_index < 0 || released_index >= sample_capacity_) {
        return InsStatus::attempted_to_release_invalid_index;
    }
    free_list_index_++;
    MeasurementFreeList[free_list_index_] = released_index;
    recorded_measurements_--;

    return InsStatus::ok;
};


InsStatus DelayedStateFR::nextMeasurement(int16_t& cur_mea_index) {
    int16_t cur_mea_index = MeasurementList[cur_mea_index].next_measurement_index;
    if (cur_mea_index == -1) return InsStatus::indexed_past_last_measurement;

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

// Method to return the timestamp of a state checkpoint
uint64_t DelayedStateFR::getStateTime(int16_t state_index) {
    return MeasurementList[StateBuffer[state_index].measurement_index].measurement.timestamp;
};

// Method for safely adding a new state/checkpoint
InsStatus DelayedStateFR::addNewState(const FullState& new_state) {

    // check if StateBuffer is full, if so rmLastState
    if (recorded_states_ == state_capacity_) {
        InsStatus rm_status = removeLastState();
        if (rm_status != InsStatus::ok) return rm_status;
    }

    int16_t newest_state_ = (newest_state_ + 1) % state_capacity_;
    recorded_states_++;
    
    StateBuffer[newest_state_].saved_state = new_state;
    StateBuffer[newest_state_].measurement_index = -1;
    
    return InsStatus::ok;
};

// Method to remove the oldest state and all the measurements dependent on it.
InsStatus DelayedStateFR::removeLastState(void) {
    if (oldest_state_ < 0 || oldest_state_ >= state_capacity_) return InsStatus::invalid_state_index_removeLastState;

    int16_t next_measurement;

    int16_t cur_measurement = StateBuffer[oldest_state_].measurement_index;
    if (cur_measurement < 0 || cur_measurement >= sample_capacity_) return InsStatus::unlinked_state_removeLastState;

    MeasurementList[cur_measurement].state_index = -1;

    // loop until a measurement linked to another state is found.
    while (cur_measurement >= 0 && cur_measurement < sample_capacity_) {
        if (MeasurementList[cur_measurement].state_index != -1) break;
        
        // clear the current measurement and proceed to the next one.
        next_measurement = MeasurementList[cur_measurement].next_measurement_index;
        MeasurementList[cur_measurement].next_measurement_index = -1;
        MeasurementList[cur_measurement].state_index = -1;
        releaseMeasurementIndex(cur_measurement);

        cur_measurement = next_measurement;
    }

    // remove the last (oldest) state once the measurements are removed.
    StateBuffer[oldest_state_].measurement_index = -1;
    oldest_state_ = (oldest_state_ + 1) % state_capacity_;
    recorded_states_--;

    return InsStatus::ok;
};

// Method to insert a delayed measurement into the MeasurementList
InsStatus DelayedStateFR::insertMeasurementAfter(const int16_t parent_measurement_i, const InternalMeasurement& new_measurement) {
  
    int16_t new_index;
    InsStatus acquire_status = acquireMeasurementIndex(new_index);
    if (acquire_status != InsStatus::ok) return acquire_status;

    // add new measurement to the measurement linked list, with its child being 
    // the previous child of its new parent
    MeasurementList[new_index] = {new_measurement, MeasurementList[parent_measurement_i].next_measurement_index, -1};

    // link the parent to this new node, over-writing its prev child.
    MeasurementList[parent_measurement_i].next_measurement_index = new_index;

    // if the parent was the tail, the inserted node is the new tail.
    if (parent_measurement_i == tail_measurement_) {
        tail_measurement_ = new_index;
        measurements_since_checkpoint_++;
    }

    return InsStatus::ok;
}

}