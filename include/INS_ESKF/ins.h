/*
Author      - Calvin Gross
Start Date  - 9/27/26
Title       - Integrated Navigation System
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This is the header file that declares the user facing methods and 
            interacts with both the delayed_state_fr and eskf. It allows access
            to all ins versions.
*/

#pragma once

#include "delayed_state_fr.h"
#include "eskf.h" 
#include "ins_types.h"

#include <stdbool.h>
#include <variant>
#include <stdint.h>

namespace ins {

class Ins 
{
public:

    bool initSimV1Ins(void);

    bool applySimV1Measurement(const InternalMeasurement& new_measurement);

    bool getCurrentState(NStateVec& nominal_state);


private:
    bool applyGeodeticGNSSCorrection();

    bool applyECEFGNSSCorrection();
};

}