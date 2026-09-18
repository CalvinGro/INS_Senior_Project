/*
Author      - Calvin Gross
Date        - 9/13/26
Modified    - 9/14/26
Modified    - 9/15/26
Title       - Delayed State Buffer Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header function for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/


#pragma once

#include "eskf.hpp" 

#include <stdbool.h>
#include <variant>
#include <stdint.h>


class DelayedStateBuffer
{
private:
    static const uint16_t MeasurementCapacity = 256;
    static const uint16_t StateCapacity = MeasurementCapacity / 5;

    // Free-list used to track indices available in the fixed-capacity measurement linked list.
    std::array<int16_t, MeasurementCapacity> MeasurementFreeList = [] {
        std::array<int16_t, MeasurementCapacity> indices{};
        for (int16_t i = 0; i < MeasurementCapacity; i++) {
            indices[i] = i;
        }
        return indices;
    }();

    int16_t free_list_index = MeasurementCapacity - 1;
    int16_t acquireMeasurementIndex(void);
    void releaseMeasurementIndex(int16_t);

public:
    enum class MeasurementStatus {
        out_of_bounds,
        delayed,
        on_time
    };
    
    struct StateCheckpoint {
        Eskf::CovarianceMatrix covar_matrix;
        Eskf::NominalState nominal_state;
        int16_t measurement_index;
    };

    struct MeasurementNode {
        Eskf::Measurement measurement;
        int16_t next_measurement_index;
        int16_t state_index;
    };


    // Measurement Fixed-Size Linked-List
    std::array<DelayedStateBuffer::MeasurementNode, MeasurementCapacity> MeasurementList;
    int16_t tail_measurement = -1;
    int16_t recorded_measurements = 0;

    // State Circular Buffer
    std::array<DelayedStateBuffer::StateCheckpoint, StateCapacity> StateBuffer;
    int16_t oldest_state = -1;
    int16_t newest_state = -1;
    int16_t recorded_states = 0;


    uint64_t DelayedStateBuffer::getStateTime(int16_t state_index);

    MeasurementStatus checkIfDelayed(uint64_t timestamp);

    int16_t getStartState(uint64_t timestamp);

    bool insertMeasurementAfter(MeasurementNode& parent_measurement, const Eskf::Measurement& new_measurement);

    bool appendMeasurement(const Eskf::Measurement& new_measurement);
    
    bool appendStateAndMesasurement(
        const Eskf::Measurement& new_measurement, 
        const Eskf::NominalState& new_nominal_state, 
        const Eskf::CovarianceMatrix& new_covariance
    );
    
    bool removeLastState(void);
};