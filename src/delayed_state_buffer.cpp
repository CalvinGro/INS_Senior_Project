/*
Author      - Calvin Gross
Date         9/13/26
Modified    - 9/14/26
Title       - Delayed State Buffer Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header function for the Delayed State Buffer class.
            This class is responsible for saving the state, covariance, and
            measurement after every measurement is processed. This allows for
            processing late correction data using a rewind and replay technique.
*/


#include "delayed_state_buffer.hpp"


// 
int16_t DelayedStateBuffer::acquireMeasurementIndex(void) {
    int16_t available_index = this->MeasurementFreeList[this->FreeListIndex];
    this->FreeListIndex--;
    return available_index;
};

void DelayedStateBuffer::releaseMeasurementIndex(int16_t released_index) {
    this->FreeListIndex++;
    this->MeasurementFreeList[this->FreeListIndex] = released_index;
};


