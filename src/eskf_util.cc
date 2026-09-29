/*
Author      - Calvin Gross
Date        - 9/28/26
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

Eigen::Quaternionf Eskf::RotationVectorToQuaternion( const Eigen::Vector3f& rotation_vector) {

    const float angle = rotation_vector.norm();

    // Avoid dividing by 0
    if (angle == 0) return Eigen::Quaternionf::Identity();

    return Eigen::Quaternionf(Eigen::AngleAxisf(angle, rotation_vector / angle));
};

}