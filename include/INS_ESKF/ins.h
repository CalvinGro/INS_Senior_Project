/*
Author      - Calvin Gross
Date        - 9/27/26
Title       - Integrated Navigation System
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header file that declares the user facing methods and 
            interacts with both the delayed_state_buffer and eskf.
*/

#pragma once

#include "delayed_state_buffer.h"
#include "eskf.h" 
#include "ins_types.h"

#include <stdbool.h>
#include <variant>
#include <stdint.h>

namespace ins {

class Ins 
{
public:

    // methods defined in eskf.cpp
    bool initEskf(void);

    bool applyMeasurement(const Measurement& new_measurement);

    bool getCurrentState(NominalState& nominal_state);

};

}