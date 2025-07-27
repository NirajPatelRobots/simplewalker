/* Predict acceleration from state., and StateEstimation.predict integrates using acceleration.
 * TODO:
 *     tilt needs to learn a coefficient of moment of inertia
 */
#ifndef SIMPLEWALKER_STATE_PREDICTION_HPP
#define SIMPLEWALKER_STATE_PREDICTION_HPP
#include "robot_state.hpp"

void predict_zero_a(const RobotState &, Eigen::Ref<Vector3f> accel_pred);
void predict_const_a(const RobotState &state, Eigen::Ref<Vector3f> accel_pred);

void predict_from_two_legs(const RobotState &state, Eigen::Ref<Vector3f> accel_pred); // TODO state needs legs

// tilting acceleration of tilt_point around rot_point in world coordinates
void predict_point_tilt(const Vector3f &tilt_point, const Vector3f &rot_point, Eigen::Ref<Vector3f> accel_pred);

// tilting acceleration of tilt_point around line defined by two points in world coordinates
void predict_line_tilt(const Vector3f &tilt_point, const Vector3f &rot_axis_p1, const Vector3f &rot_axis_p2, Eigen::Ref<Vector3f> accel_pred);


#endif //SIMPLEWALKER_STATE_PREDICTION_HPP
