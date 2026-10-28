/*
Author      - Calvin Gross
Start Date  - 10/23/26
Title       - Ins Simulation version 1 Header
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the interface to apply abstracted measurements of
            simple states without any sensor nuance to the Eskf. It provides a 
            minimal initialization method and state getter.
*/


#pragma once

#include "delayed_state_fr.h"
#include "eskf.h" 
#include "ins_types.h"

#include <stdbool.h>
#include <variant>
#include <stdint.h>

#define LIST_OF_ERRORS_SIZE 500


namespace ins {

class InsSimV1
{

    Eskf eskf = Eskf();
    DelayedStateFR delayed_fr = DelayedStateFR(eskf);

    InsStatus list_of_errors[LIST_OF_ERRORS_SIZE];
    uint16_t loe_i = 0;

public:

    InsStatus initSimV1Ins(void);

    InsStatus getCurrentState(NStateVec& nominal_state);

    void receiveV1AccelGyro(const AGExternalV1Sample& ex_ag_sample);
    void receiveV1Baro(const BaroExternalV1Sample& ex_baro_sample);
    void receiveV1Mag(const MagExternalV1Sample& ex_mag_sample);
    void receiveV1Gnss(const GnssExternalV1Sample& ex_gnss_sample);

private:
    void InsSimV1::processV1Measurement(const InternalMeasurement& measurement);

    void InsSimV1::appendError(const InsStatus error);
};
}