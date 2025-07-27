#include <gtest/gtest.h>
#include "state_prediction.hpp"

const float TOLERANCE = 1e-6;

RobotState get_random_state() {
    RobotState state;
    state.vect.setRandom();
    state.acceleration.setRandom();
    state.calculate();
    return state;
}

bool MatrixIsClose(const MatrixXf &lhs, const MatrixXf &rhs) {
    return lhs.isApprox(rhs, TOLERANCE);
}

bool MatrixIsCloseToZero(const MatrixXf &test) {
    return test.squaredNorm() < TOLERANCE;
}

TEST(StatePrediction, TestZeroAccel) {
    RobotState state = get_random_state();
    Vector3f accel_result = Vector3f::Random();

    predict_zero_a(state, accel_result);
    ASSERT_EQ(accel_result, Vector3f::Zero());
}

TEST(StatePrediction, TestConstAccel) {
    RobotState state = get_random_state();
    Vector3f accel_result = Vector3f::Random();

    predict_const_a(state, accel_result);
    ASSERT_EQ(accel_result, state.acceleration);
}

// Tilt

TEST(StatePrediction, TestPointTiltUpright) {
    Vector3f accel_result = Vector3f::Random();
    float height = Eigen::Vector<float, 1>::Random()(0) + 2;
    Vector3f rot_point(0, 0, -height);

    predict_point_tilt(Vector3f::Zero(), rot_point, accel_result);

    ASSERT_PRED1(MatrixIsCloseToZero, accel_result);
}

TEST(StatePrediction, TestPointTilt) {
    Vector3f accel_result = Vector3f::Random(), accel_result_2 = Vector3f::Random();
    Vector3f offset =  Vector3f::Random();
    offset(UP_IDX) = offset(UP_IDX) * 0.5 - 1;  // rot point below tilt_point

    // test accel perp to "offset" vector from origin to rotation axis point
    predict_point_tilt(Vector3f::Zero(), offset, accel_result);
    float angle_between = acosf(offset.dot(accel_result) / (offset.norm() * accel_result.norm()));
    ASSERT_NEAR(angle_between, M_PI_2, TOLERANCE);

    // test distance to point doesn't affect accel
    predict_point_tilt(Vector3f::Zero(), 2 * offset, accel_result_2);
    EXPECT_PRED2(MatrixIsClose, accel_result, accel_result_2);

    // test adding a global pos to all points has no effect
    Vector3f global_pos = Vector3f::Random(), accel_result_3 = Vector3f::Random();
    predict_point_tilt(global_pos, offset + global_pos, accel_result_3);
    EXPECT_PRED2(MatrixIsClose, accel_result, accel_result_3);
}

TEST(StatePrediction, TestPointTiltHorizontal) {
    Vector3f accel_result = Vector3f::Random(), accel_result_2 = Vector3f::Random();
    Vector3f offset = Vector3f::Random();
    offset(UP_IDX) = 0;

    predict_point_tilt(Vector3f::Zero(), offset, accel_result);
    // check accel is gravitational acceleration
    EXPECT_PRED2(MatrixIsClose, accel_result, GRAVITY_ACCEL);

    // test switching points to opposite side doesn't affect accel
    predict_point_tilt(Vector3f::Zero(), -offset, accel_result_2);
    EXPECT_PRED2(MatrixIsClose, accel_result_2, GRAVITY_ACCEL);
}

// world frame
void randomLegPositions(float offset_dist, Vector3f &offset, Vector3f &left_point, Vector3f &right_point) {
    float angle = Eigen::Vector<float, 1>::Random()(0) * M_PI_2;
    float height = Eigen::Vector<float, 1>::Random()(0) + 2;
    Matrix3f rotation {Eigen::AngleAxisf(angle, Vector3f{0,0,1})};
    offset = rotation * Vector3f {offset_dist, 0, -height};
    float left_dist_out = Eigen::Vector<float, 1>::Random()(0), right_dist_out = Eigen::Vector<float, 1>::Random()(0);
    left_point =  rotation * Vector3f{0,  left_dist_out,  0} + offset;
    right_point = rotation * Vector3f{0, -right_dist_out, 0} + offset;
}

TEST(StatePrediction, TestLineTiltUpright) {
    Vector3f accel_result = Vector3f::Random();
    Vector3f left_point, right_point, offset;
    randomLegPositions(0, offset, left_point, right_point);

    predict_line_tilt(Vector3f::Zero(), left_point, right_point, accel_result);

    ASSERT_PRED1(MatrixIsCloseToZero, accel_result);
}

TEST(StatePrediction, TestLineTilt) {
    Vector3f accel_result = Vector3f::Random(), accel_result_2 = Vector3f::Random();
    Vector3f left_point, right_point, offset;
    float offset_dist = Eigen::Vector<float, 1>::Random()(0) * .5 + .7;
    randomLegPositions(offset_dist, offset, left_point, right_point);
    EXPECT_NEAR(offset_dist, project_zeroZ(offset).norm(), TOLERANCE);

    // test accel perp to "offset" vector from origin to rotation axis line
    predict_line_tilt(Vector3f::Zero(), left_point, right_point, accel_result);
    float angle_between = acosf(offset.dot(accel_result) / (offset.norm() * accel_result.norm()));
    ASSERT_NEAR(angle_between, M_PI_2, TOLERANCE);

    // test distance to points doesn't affect accel
    predict_line_tilt(Vector3f::Zero(), 2 * left_point, 2 * right_point, accel_result_2);
    EXPECT_PRED2(MatrixIsClose, accel_result, accel_result_2);

    // test adding a global pos to all points has no effect
    Vector3f global_pos = Vector3f::Random(), accel_result_3 = Vector3f::Random();
    predict_line_tilt(global_pos, left_point + global_pos, right_point + global_pos, accel_result_3);
    EXPECT_PRED2(MatrixIsClose, accel_result, accel_result_3);
}

TEST(StatePrediction, TestLineTiltHorizontal) {
    Vector3f accel_result = Vector3f::Random(), accel_result_2 = Vector3f::Random();
    Vector3f left_point, right_point, offset, tilt_point = Vector3f::Zero();
    float offset_dist = Eigen::Vector<float, 1>::Random()(0) * .5 + .7;
    randomLegPositions(offset_dist, offset, left_point, right_point);
    tilt_point(UP_IDX) = left_point(UP_IDX); // tilt point at same height as rotation points

    predict_line_tilt(tilt_point, left_point, right_point, accel_result);
    // check accel is gravitational acceleration
    EXPECT_PRED2(MatrixIsClose, accel_result, GRAVITY_ACCEL);

    // test switching points to opposite side doesn't affect accel
    predict_line_tilt(-tilt_point, -left_point, -right_point, accel_result_2);
    EXPECT_PRED2(MatrixIsClose, accel_result_2, GRAVITY_ACCEL);
}
