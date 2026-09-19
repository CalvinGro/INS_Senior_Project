/*
Author      - Calvin Gross
Date        - 9/18/26
Title       - Error State Kalman Filter Correction Methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the methods to correct the state vector and covariance
            matrix based on gnss, magnetometer, or barometer measurements.
*/


#pragma once


#include "eskf.hpp" 


bool Eskf::apply_mag_correction(const MagCorrectionData& mag_sample, uint16_t dt) {

};

bool Eskf::apply_gnss_correction(const GnssCorrectionData& gnss_sample, uint16_t dt) {

};

bool Eskf::apply_baro_correction(const BaroCorrectionData& baro_sample, uint16_t dt) {
    
};