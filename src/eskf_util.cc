/*
Author      - Calvin Gross
Start Date  - 9/28/26
Title       - Error State Kalman Filter Utility Methods
Project     - Integrated Navigation System (GNSS + IMU) -- Senior Project --
Description - This file defines the supporting methods for the prediction and
            correction steps in the Eskf class. 
*/

#include "ins_types.h"
#include "eskf.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ins {

Eigen::Quaternionf Eskf::RotationVectorToQuaternion(const Eigen::Vector3f& rotation_vector) {

    const float angle = rotation_vector.norm();

    // Avoid dividing by 0
    if (angle == 0) return Eigen::Quaternionf::Identity();

    return Eigen::Quaternionf(Eigen::AngleAxisf(angle, rotation_vector / angle));
};

// Method to inject the observed/calculated error-state into the nominal-state after
// receiving a measurement.
void Eskf::CorrectionInjection(const Eigen::Vector<float,16>& es, NStateVec& ns) {
    ns.position += es.segment<3>(0);
    ns.velocity += es.segment<3>(3);
    // angular_position calculation assumes the orientation correction is a rotation vector
    ns.angular_position = (RotationVectorToQuaternion(es.segment<3>(6)) * ns.angular_position).normalized(); 
    ns.accel_bias += es.segment<3>(9);
    ns.gyro_bias += es.segment<3>(12);
    ns.baro_bias += es[15];
};

}