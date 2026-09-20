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


bool Eskf::applyMagCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const MagCorrectionData& mag_sample,
     uint64_t dt
    ) {

};

bool Eskf::applyGnssCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const GnssCorrectionData& gnss_sample, 
        uint64_t dt
    ) {

};

bool Eskf::applyBaroCorrection(
        NominalState& nom_state, 
        CovarianceMatrix& covar, 
        const BaroCorrectionData& baro_sample, 
        uint64_t dt
    ) {

};