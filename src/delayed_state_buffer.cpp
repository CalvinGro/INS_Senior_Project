/*
Author      - Calvin Gross
Date         9/13/26
Modified    - 9/14/26
Modified    - 9/15/26
Modified    - 9/18/26
Title       - Delayed State Buffer Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header function for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/


#include "delayed_state_buffer.hpp"


// Free-List Methods

int16_t DelayedStateBuffer::acquireMeasurementIndex(void) {
    if (this->free_list_index == -1) return -1;
    int16_t available_index = this->MeasurementFreeList[this->free_list_index];
    this->free_list_index--;
    return available_index;
};


void DelayedStateBuffer::releaseMeasurementIndex(int16_t released_index) {
    this->free_list_index++;
    this->MeasurementFreeList[this->free_list_index] = released_index;
};



// Fixed-Capacity Linked-List for storing all measurements 
// and Circular Buffer for storing state checkpoints

uint64_t DelayedStateBuffer::getStateTime(int16_t state_index) {
    return MeasurementList[StateBuffer[state_index].measurement_index].measurement.timestamp;
};


bool  DelayedStateBuffer::removeLastState(void) {
    if (oldest_state < 0 || oldest_state >= StateCapacity) return false;

    int16_t next_measurement;

    int16_t cur_measurement = StateBuffer[oldest_state].measurement_index;
    if (cur_measurement < 0 || cur_measurement >= MeasurementCapacity) return false;

    MeasurementList[cur_measurement].state_index = -1;

    // loop until a measurement linked to another state is found.
    while (cur_measurement >= 0 && cur_measurement < MeasurementCapacity) {
        if (MeasurementList[cur_measurement].state_index != -1) break;
        
        // clear the current measurement and proceed to the next one.
        next_measurement = MeasurementList[cur_measurement].next_measurement_index;
        MeasurementList[cur_measurement].next_measurement_index = -1;
        MeasurementList[cur_measurement].state_index = -1;
        releaseMeasurementIndex(cur_measurement);

        cur_measurement = next_measurement;
    }

    // remove the last (oldest) state once the measurements are removed.
    StateBuffer[oldest_state].measurement_index = -1;
    if (oldest_state == (StateCapacity-1)) oldest_state = 0;
    else oldest_state++;
    recorded_states--;

    return true;
};


DelayedStateBuffer::MeasurementStatus DelayedStateBuffer::checkIfDelayed(uint64_t timestamp) {

    // check if timestamp is on-time (the newest timestamp)
    if(tail_measurement == -1 || MeasurementList[tail_measurement].measurement.timestamp <= timestamp) {
        return MeasurementStatus::on_time;
    }

    // check if timestamp is out-of-bounds (past the saved measurements and states)
    uint64_t oldest_timestamp = getStateTime(oldest_state);
    if (oldest_timestamp > timestamp) {
        return MeasurementStatus::out_of_bounds;
    }

    // otherwise the timestamp is somewhere within the saved measurements
    return MeasurementStatus::delayed;
};


int16_t  DelayedStateBuffer::getStartState(uint64_t timestamp) {
    
    // lambda function for transforming bi-search index into circular buffer index
    std::function<int16_t(int16_t)> to_cb_index = [=](int16_t i) {
        return (i + oldest_state) % StateCapacity;
    };

    // binary search for starting state 
    int16_t i = 0;
    int16_t j = recorded_states - 1;
    if (j == -1) return -1;

    while((i + 1) < j) {
        int16_t mid = i + (j - i) / 2;
        uint64_t mid_time = getStateTime(to_cb_index(mid));
        
        if (timestamp < mid_time) {
            j = mid;
        } else if (timestamp > mid_time) {
            i = mid;
        } else {
            return to_cb_index(mid);
        }
    };

    // return the state found
    if (timestamp >= getStateTime(to_cb_index(j))) {
        return to_cb_index(j);
    } else if (timestamp >= getStateTime(to_cb_index(i))) {
        return to_cb_index(i);
    } else {
        return -1;
    }
};


bool DelayedStateBuffer::insertMeasurementAfter(MeasurementNode& parent_measurement, const Eskf::Measurement& new_measurement) {
  
    // if measurement linked-list has space.
    int16_t new_index = acquireMeasurementIndex();
    if (new_index == -1) {
        bool rm_state_status = removeLastState();
        if (!rm_state_status) return false;

        new_index = acquireMeasurementIndex();
        if (new_index == -1) return false;
    }

    // add new measurement to the measurement linked list, with its child being 
    // the previous child of its new parent
    MeasurementList[new_index] = {new_measurement, parent_measurement.next_measurement_index, -1};

    // link the parent to this new node, over-writing its prev child.
    parent_measurement.next_measurement_index = new_index;

    // if the parent was the tail, the inserted node is the new tail.
    if (MeasurementList[new_index].next_measurement_index == -1) {
        tail_measurement = new_index;
    }

    return true;
}


bool DelayedStateBuffer::appendMeasurement(const Eskf::Measurement& new_measurement) {

    // if measurement linked-list has space.
    int16_t new_index = acquireMeasurementIndex();
    if (new_index == -1) {
        bool rm_state_status = removeLastState();
        if (!rm_state_status) return false;

        new_index = acquireMeasurementIndex();
        if (new_index == -1) return false;
    }

    // add measurement node to the front
    MeasurementList[new_index] = {new_measurement, -1, -1};
    
    // link to prev tail (unless it is the first measurement added)
    if (tail_measurement != -1) {
        MeasurementList[tail_measurement].next_measurement_index = new_index;
    }
    tail_measurement = new_index;
    
    return true;
};



bool DelayedStateBuffer::appendStateAndMesasurement(
    const Eskf::Measurement& new_measurement, 
    const Eskf::NominalState& new_nominal_state, 
    const Eskf::CovarianceMatrix& new_covariance
) {

    bool meaStatus = appendMeasurement(new_measurement);
    if (!meaStatus) return false;

    int16_t next_new_state = (newest_state + 1) % StateCapacity;

    // if first state and measurement added
    if (oldest_state == -1) {
        oldest_state++;
    // else if circular buffer is full
    } else if (oldest_state == next_new_state) {
        bool rm_state_status = removeLastState();
        if (!rm_state_status) return false;
    }

    // not set right away in the case of removeLastState() failing
    newest_state = next_new_state;

    MeasurementList[tail_measurement].state_index = newest_state;
    StateBuffer[newest_state] = {new_covariance, new_nominal_state, tail_measurement};
    recorded_states++;
    return true;
};

