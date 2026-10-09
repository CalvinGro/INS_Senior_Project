/*
Author      - Calvin Gross
Date        - 9/28/26
Modified    - 10/8/26
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

/*  This method is used after receiving a correction measurement. It propagates the current
    nominal-state and error-state covariance to the timestamp of the correction measurement.

*/
bool Eskf::PropgateStatePreCorrection(NStateVec& nom_state, ESCovarMtx& covar, uint64_t dt) {
    
    float dt_seconds = static_cast<float>(dt) * 1e-6f;

    // Calculate rotation matrix R for the previous orientation (backward integration).
    // Normalization is performed to prevent tiny rounding errors from growing.
    Eigen::Matrix3f R = nom_state.angular_position.normalized().toRotationMatrix();

    nom_state.position = nom_state.position + nom_state.velocity * dt_seconds;
}
}