/*
Author      - Calvin Gross
Date         9/13/26
Title       - Delayed State Buffer Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header function for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/


#include "eskf.hpp" 

#include <variant>
#include <stdint.h>



#pragma once


class DelayedStateBuffer
{
private:
    uint16_t start;

public:
    struct SavedMeasurementState {
        Eskf::CovarianceMatrix covar_matrix;
        Eskf::NominalState nominal_state;
        Eskf::Measurement measurement;
    };

    uint16_t getStartIndex(void);
    uint16_t findIndex(uint64_t timestamp);
    void pushFront(const SavedMeasurementState& measurement_state);
};