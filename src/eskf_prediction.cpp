/*
Author      - Calvin Gross
Date        - 9/18/26
Title       - Error State Kalman Filter Prediction Methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the methods to predict/update the state vector and
            covariance matrix based on accelerometer or gyroscope measurements.
*/


#pragma once


#include "eskf.hpp" 


bool Eskf::apply_accel_prediction(const AccelUpdateData& accel_sample, uint16_t dt) {

};


bool Eskf::apply_gyro_prediction(const GyroUpdateData& gyro_sample, uint16_t dt) {

};