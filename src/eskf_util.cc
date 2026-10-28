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

#include <variant>

namespace ins {

InsStatus Eskf::applyMeasurement(const InternalMeasurement& new_measurement) {
    return applyMeasurement(new_measurement, cur_state);
};

// Overloaded is used for applying the correct lambda to each type of measurement in Measurement
template<typename... MeaLambda>
struct Overloaded : MeaLambda... {
    using MeaLambda::operator()...;
};

// tells the compiler how to use the template args
template<typename... MeaLambda>
Overloaded(MeaLambda...) -> Overloaded<MeaLambda...>;

InsStatus Eskf::applyMeasurement(const InternalMeasurement& new_measurement, FullState& prev_state) {
    if (prev_state.last_timestamp > new_measurement.timestamp) return InsStatus::timestamps_out_of_order_applyMeasurement;
    uint64_t dt = new_measurement.timestamp - prev_state.last_timestamp;

    InsStatus apply_status = std::visit(
        Overloaded{
            [this, &prev_state, dt](const AGInternalSample& ag_sample) {
                return applyAccelGyroPredict(dt, ag_sample, prev_state);
            },
            [this, &prev_state, dt](const GnssInternalSample& gnss_sample) {
                return applyGnssCorrection(dt, gnss_sample, prev_state);
            },
            [this, &prev_state, dt](const MagInternalSample& mag_sample) {
                return applyMagCorrection(dt, mag_sample, prev_state);
            },
            [this, &prev_state, dt](const BaroInternalSample& baro_sample) {
                return applyBaroCorrection(dt, baro_sample, prev_state);
            }
        }, new_measurement.data
    );

    if (apply_status == InsStatus::ok) {
        prev_state.last_timestamp = new_measurement.timestamp;
    }
    return apply_status;
};


Eigen::Quaternionf Eskf::rotationVectorToQuaternion(const Eigen::Vector3f& rotation_vector) {

    const float angle = rotation_vector.norm();

    // Avoid dividing by 0
    if (angle == 0) return Eigen::Quaternionf::Identity();

    return Eigen::Quaternionf(Eigen::AngleAxisf(angle, rotation_vector / angle));
};

// Method to inject the observed/calculated error-state into the nominal-state after
// receiving a measurement.
void Eskf::correctionInjection(const Eigen::Vector<float,16>& es, NStateVec& ns) {
    ns.position += es.segment<3>(0);
    ns.velocity += es.segment<3>(3);
    // angular_position calculation assumes the orientation correction is a rotation vector
    ns.angular_position = (rotationVectorToQuaternion(es.segment<3>(6)) * ns.angular_position).normalized(); 
    ns.accel_bias += es.segment<3>(9);
    ns.gyro_bias += es.segment<3>(12);
    ns.baro_bias += es[15];
};

}