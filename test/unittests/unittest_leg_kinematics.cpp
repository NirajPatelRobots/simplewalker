/* unittests for leg kinematic equations for simplewalker
Created by Niraj, Dec 2022.
TODO:
 */


#include <gtest/gtest.h>
#include "leg_kinematics.hpp"

const float TOLERANCE = 1e-6;
const float TEST_RADIUS { 0.01 };
const float FOOT_LENGTH { 0.03 };


bool MatrixIsClose(const Eigen::MatrixXf &lhs, const Eigen::MatrixXf &rhs) {
    return lhs.isApprox(rhs, TOLERANCE);
}

bool MatrixIsCloseToZero(const Eigen::MatrixXf &test) {
    return test.squaredNorm() < TOLERANCE;
}

inline float randomFloat() {return Eigen::Vector<float, 1>::Random()(0);}

/************************ ForwardKinematics ************************/
TEST(ForwardKinematics, LegStraightDown) {
    Vector3f p_leg = Vector3f::Random();
    Vector3f leg_angles = Vector3f::Zero();
    forward_kinematics(p_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, p_leg, -(SHIN_LENGTH + THIGH_LENGTH) * UP_DIR);
}

TEST(ForwardKinematics, LegStraightOutSideways) {
    Vector3f p_leg = Vector3f::Random();
    Vector3f leg_angles {M_PI_2, 0, 0};
    forward_kinematics(p_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, p_leg.cwiseAbs(), (SHIN_LENGTH + THIGH_LENGTH) * LEFT_DIR);
}

TEST(ForwardKinematics, LegForward) {
    Vector3f p_leg = Vector3f::Random();
    Vector3f leg_angles {0, M_PI_2, M_PI_2};
    forward_kinematics(p_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, p_leg, (SHIN_LENGTH + THIGH_LENGTH) * FORWARD_DIR);
}

TEST(ForwardKinematics, KneeRightAngle) {
    Vector3f p_leg = Vector3f::Random();
    Vector3f leg_angles {0, 0, -M_PI_2};
    forward_kinematics(p_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, p_leg, -SHIN_LENGTH * UP_DIR - THIGH_LENGTH * FORWARD_DIR);
}

/************************ ForwardKinematics_R ************************/
TEST(ForwardKinematics_R, LegStraightDown) {
    Matrix3f R_leg = Matrix3f::Random();
    Vector3f leg_angles = Vector3f::Zero();
    forward_kinematics_R(R_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, R_leg, Matrix3f::Identity());
}

TEST(ForwardKinematics_R, LegStraightOutSideways) {
    Matrix3f R_leg = Matrix3f::Random();
    Vector3f leg_angles {M_PI_2, 0, 0};
    forward_kinematics_R(R_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, (R_leg * LEFT_DIR).cwiseAbs(), UP_DIR);
    EXPECT_PRED2(MatrixIsClose, R_leg * FORWARD_DIR, FORWARD_DIR);
    EXPECT_PRED2(MatrixIsClose, (R_leg * UP_DIR).cwiseAbs(), LEFT_DIR);
}

TEST(ForwardKinematics_R, LegForward) {
    Matrix3f R_leg = Matrix3f::Random();
    Vector3f leg_angles {0, M_PI_2, M_PI_2};
    forward_kinematics_R(R_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, R_leg * LEFT_DIR, LEFT_DIR);
    EXPECT_PRED2(MatrixIsClose, R_leg * FORWARD_DIR, UP_DIR);
    EXPECT_PRED2(MatrixIsClose, R_leg * UP_DIR, -FORWARD_DIR);
}

TEST(ForwardKinematics_R, KneeRightAngle) {
    Matrix3f R_leg = Matrix3f::Random();
    Vector3f leg_angles {0, 0, -M_PI_2};
    forward_kinematics_R(R_leg, leg_angles);
    EXPECT_PRED2(MatrixIsClose, R_leg * LEFT_DIR, LEFT_DIR);
    EXPECT_PRED2(MatrixIsClose, R_leg * FORWARD_DIR, -UP_DIR);
    EXPECT_PRED2(MatrixIsClose, R_leg * UP_DIR, FORWARD_DIR);
}

/************************ CalcFootSpringAngle ************************/
TEST(CalcFootSpringAngle, Flat_NotTouchGround) {
    const float height = 1;
    const Vector3f unsprung_point = UP_DIR * height;
    Vector3f sprung_point = unsprung_point + FOOT_LENGTH * FORWARD_DIR;
    const Matrix3f R_body_leg = Matrix3f::Identity();

    float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
    EXPECT_EQ(angle, 0);
    // sprung_point should not change
    EXPECT_PRED2(MatrixIsClose, sprung_point, UP_DIR * height + FOOT_LENGTH * FORWARD_DIR);
}

TEST(CalcFootSpringAngle, Flat_OnGround) {
    const Vector3f unsprung_point = UP_DIR * TEST_RADIUS;
    Vector3f sprung_point = unsprung_point + FOOT_LENGTH * FORWARD_DIR;
    const Matrix3f R_body_leg = Matrix3f::Identity();

    float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
    EXPECT_EQ(angle, 0);
    // sprung_point should not change
    EXPECT_PRED2(MatrixIsClose, sprung_point, UP_DIR * TEST_RADIUS + FOOT_LENGTH * FORWARD_DIR);
}

// above ground, foot angled at 45deg
TEST(CalcFootSpringAngle, Angled_NotTouchGround) {
    const float height = 1;
    const Vector3f unsprung_point = UP_DIR * height;
    Vector3f sprung_point = unsprung_point + FOOT_LENGTH / sqrt(2) * (FORWARD_DIR - UP_DIR);
    const Matrix3f R_body_leg = Eigen::AngleAxisf(M_PI / 4, LEFT_DIR).toRotationMatrix();
    // Confirm test parameters are good: check R_body_leg matches sprung point
    EXPECT_PRED2(MatrixIsClose, sprung_point - unsprung_point, R_body_leg * FOOT_LENGTH * FORWARD_DIR);

    float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
    EXPECT_EQ(angle, 0);
    // distance to foot point should not change
    EXPECT_NEAR((sprung_point - unsprung_point).norm(), FOOT_LENGTH, TOLERANCE);
    // sprung_point should not change since it does not touch the ground
    EXPECT_PRED2(MatrixIsClose, sprung_point, unsprung_point + FOOT_LENGTH / sqrt(2) * (FORWARD_DIR - UP_DIR));
}

// angled at 45deg, unsprung_point touching ground
TEST(CalcFootSpringAngle, Angled_OnGround) {
    const Vector3f unsprung_point = UP_DIR * TEST_RADIUS;
    Vector3f sprung_point = unsprung_point + FOOT_LENGTH / sqrt(2) * (FORWARD_DIR - UP_DIR);
    const Matrix3f R_body_leg = Eigen::AngleAxisf(M_PI / 4, LEFT_DIR).toRotationMatrix();

    float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
    // distance to foot point should not change
    EXPECT_NEAR((sprung_point - unsprung_point).norm(), FOOT_LENGTH, TOLERANCE);
    // foot point should be on ground
    EXPECT_NEAR(sprung_point[UP_IDX], TEST_RADIUS, TOLERANCE);
    // angle should be 45deg
    EXPECT_NEAR(angle, M_PI / 4, TOLERANCE);
}

// start angled at 45deg, barely touching ground, lower until both points touching ground
TEST(CalcFootSpringAngle, Angled_LowerToGround) {
    float last_angle = -1, start_height = FOOT_LENGTH / sqrt(2) + TEST_RADIUS;
    int height_div = 8;
    const Matrix3f R_body_leg = Eigen::AngleAxisf(M_PI / 4, LEFT_DIR).toRotationMatrix();
    for (float height = start_height; height > TEST_RADIUS; height -= FOOT_LENGTH / sqrt(2) / height_div ) {
        const Vector3f unsprung_point = UP_DIR * height;
        Vector3f sprung_point = unsprung_point + FOOT_LENGTH / sqrt(2) * (FORWARD_DIR - UP_DIR);

        float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
//        cout << "Height: " << height << " Angle: " << angle << "\n";
        EXPECT_GT(angle, last_angle); // angle will increase as foot is lowered
        last_angle = angle;
        // distance to foot point should not change
        EXPECT_NEAR((sprung_point - unsprung_point).norm(), FOOT_LENGTH, TOLERANCE);
        // foot point should be on ground
        EXPECT_NEAR(sprung_point[UP_IDX], TEST_RADIUS, TOLERANCE);
        // angle between sprung_point and initial sprung_point should be = returned angle
        const Vector3f p_o = sprung_point - unsprung_point;
        const Vector3f p_i_unit = R_body_leg * FORWARD_DIR;
        EXPECT_NEAR(angle, std::acos(p_o.dot(p_i_unit) / FOOT_LENGTH), TOLERANCE);
    }
}

// hip angle out -> spring axis angled
// hip out angle = knee angle = 45deg, not touch ground
TEST(CalcFootSpringAngle, ComplexAngled_NotTouchGround) {
    const float height = 1;
    const Vector3f unsprung_point = UP_DIR * height;
    const Matrix3f R_body_leg = (Eigen::AngleAxisf(M_PI / 4, FORWARD_DIR)
                               * Eigen::AngleAxisf(M_PI / 4, LEFT_DIR)).toRotationMatrix();
    Vector3f sprung_point = unsprung_point + R_body_leg * FOOT_LENGTH * FORWARD_DIR;

    float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
    EXPECT_EQ(angle, 0);
    // sprung_point should not change since it does not touch the ground
    EXPECT_PRED2(MatrixIsClose, sprung_point, unsprung_point + R_body_leg * FOOT_LENGTH * FORWARD_DIR);
}


// hip out angle = knee angle = 45deg, unsprung_point touching ground
TEST(CalcFootSpringAngle, ComplexAngled_OnGround) {
    const float height = TEST_RADIUS;
    const Vector3f unsprung_point = UP_DIR * height;
    const Matrix3f R_body_leg = (Eigen::AngleAxisf(M_PI / 4, FORWARD_DIR)
                                 * Eigen::AngleAxisf(M_PI / 4, LEFT_DIR)).toRotationMatrix();
    Vector3f sprung_point = unsprung_point + R_body_leg * FOOT_LENGTH * FORWARD_DIR;

    float angle = calc_foot_spring_angle(sprung_point, unsprung_point, R_body_leg, TEST_RADIUS);
    // distance to foot point should not change
    EXPECT_NEAR((sprung_point - unsprung_point).norm(), FOOT_LENGTH, TOLERANCE);
    // foot point should be on ground
    EXPECT_NEAR(sprung_point[UP_IDX], TEST_RADIUS, TOLERANCE);
    // angle between sprung_point and initial sprung_point should be = returned angle
    const Vector3f p_o = sprung_point - unsprung_point;
    const Vector3f p_i_unit = R_body_leg * FORWARD_DIR;
    EXPECT_NEAR(angle, std::acos(p_o.dot(p_i_unit) / FOOT_LENGTH), TOLERANCE);
}
