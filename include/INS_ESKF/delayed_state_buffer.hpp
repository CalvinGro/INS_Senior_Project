/*
Author      - Calvin Gross
Date        - 9/13/26
Modified    - 9/14/26
Title       - Delayed State Buffer Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header function for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/


#pragma once

#include "eskf.hpp" 

#include <variant>
#include <stdint.h>


class DelayedStateBuffer
{
private:
    static const uint16_t MeasurementCapacity = 256;
    static const uint16_t StateCapacity = DelayedStateBuffer::MeasurementCapacity / 5;

    // Free-list used to track indices available in the fixed-capacity measurement linked list.
    std::array<int16_t, DelayedStateBuffer::MeasurementCapacity> MeasurementFreeList = [] {
        std::array<int16_t, DelayedStateBuffer::MeasurementCapacity> indices{};
        for (int16_t i = 0; i < DelayedStateBuffer::MeasurementCapacity; i++) {
            indices[i] = i;
        }
        return indices;
    }();

    int16_t FreeListIndex = MeasurementCapacity - 1;
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
        uint64_t timestamp;
    };

    struct MeasurementNode {
        Eskf::Measurement measurement;
        int16_t next_measurement_index;
        int16_t state_index;
    };

    MeasurementStatus checkIfDelayed(uint64_t timestamp);

    uint16_t getStartState(Eskf::NominalState& nominal_state, Eskf::CovarianceMatrix& covariance);

    void insertMeasurement(Eskf::Measurement new_measurement);
    
    void insertStateAndMesasurement(
        Eskf::Measurement& new_measurement, 
        Eskf::NominalState& new_nominal_state, 
        Eskf::CovarianceMatrix& new_covariance
    );
    
    void removeLastState(void);
};