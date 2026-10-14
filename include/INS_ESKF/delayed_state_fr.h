/*
Author      - Calvin Gross
Start Date  - 10/13/26
Title       - Delayed State Filter Recalculator Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header file for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/


#pragma once

#include "ins_types.h"
#include "eskf.h" 

#include <stdbool.h>
#include <variant>
#include <stdint.h>

#define SAMPLE_CAP 70
#define SAMPLES_PER_STATE 5


namespace ins {

class DelayedStateFR
{
public:
    DelayedStateFR(Eskf& eskf)
        : eskf_(eskf) {}

    InsStatus insertMeasurement(const Measurement& new_measurement);

    DelayStatus checkIfDelayed(const uint64_t timestamp);

    
private:
    const static uint64_t sample_capacity_ = SAMPLE_CAP;
    const static uint64_t state_capacity_ = sample_capacity_ / SAMPLES_PER_STATE;

    // The ESKF instance initialized by the INS 
    Eskf& eskf_;

    struct StateCheckpoint {
        FullState saved_state;
        int16_t measurement_index;
    };

    struct MeasurementNode {
        Measurement measurement;
        int16_t next_measurement_index;
        int16_t state_index;
    };


    // Measurement Fixed-Size Linked-List
    std::array<DelayedStateFR::MeasurementNode, sample_capacity_> MeasurementList;
    int16_t tail_measurement_ = -1;
    int16_t recorded_measurements_ = 0;

    // State Circular Buffer
    std::array<DelayedStateFR::StateCheckpoint, state_capacity_> StateBuffer;
    int16_t oldest_state_ = -1;
    int16_t newest_state_ = -1;
    int16_t recorded_states_ = 0;
    int16_t measurements_since_checkpoint_ = 0;

    // Free-list used to track indices available in the fixed-capacity measurement linked list.
    std::array<int16_t, sample_capacity_> MeasurementFreeList = [] {
        std::array<int16_t, sample_capacity_> indices{};
        for (int16_t i = 0; i < sample_capacity_; i++) {
            indices[i] = i;
        }
        return indices;
    }();

    int16_t free_list_index_ = sample_capacity_ - 1;

    // Methods for handling free-list
    InsStatus acquireMeasurementIndex(int16_t acquired_index);
    InsStatus releaseMeasurementIndex(const int16_t index);


    // utility methods for filter recalculation
    uint64_t getStateTime(const int16_t state_index);

    InsStatus findRecalcCheckpoint(const uint64_t timestamp, int16_t found_state_index);


};

}