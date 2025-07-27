#include "state_prediction.hpp"

void predict_zero_a(const RobotState &, Eigen::Ref<Vector3f> accel_pred) {
    accel_pred.setZero();
}

void predict_const_a(const RobotState &state, Eigen::Ref<Vector3f> accel_pred) {
    accel_pred = state.acceleration;
}

void predict_point_tilt(const Vector3f &tilt_point, const Vector3f &rot_point, Eigen::Ref<Vector3f> accel_pred) {
    Vector3f rot_radius = tilt_point - rot_point;
    // accel_pred = GRAVITY_ACCEL - (GRAVITY_ACCEL projected onto rot_radius)
    accel_pred = GRAVITY_ACCEL - (rot_radius * GRAVITY_ACCEL.dot(rot_radius) / rot_radius.squaredNorm());
}

void predict_line_tilt(const Vector3f &tilt_point, const Vector3f &rot_axis_p1, const Vector3f &rot_axis_p2, Eigen::Ref<Vector3f> accel_pred) {
    Vector3f v1 = rot_axis_p1 - rot_axis_p2, v2 = tilt_point - rot_axis_p2;
    Vector3f rot_radius = v2 - (v1 * v2.dot(v1) / v1.squaredNorm());  // v2 - (v2 projected onto v1);
    // accel_pred = GRAVITY_ACCEL - (GRAVITY_ACCEL projected onto rot_radius)
    accel_pred = GRAVITY_ACCEL - (rot_radius * GRAVITY_ACCEL.dot(rot_radius) / rot_radius.squaredNorm());
}
