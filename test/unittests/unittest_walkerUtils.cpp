/* unittests for utilities for simplewalker
Created by Niraj, Dec 2022.
TODO:
    test scalar_statistic
 */

#include <gtest/gtest.h>
#include "walkerUtils.hpp"

class JacobianRotatedAxisTest : public ::testing::Test {
public:
    Vector3f axis, input, expected_output, output;
    Matrix3f Jacobian;
    void calc_Jacobian() {Jac_rotated_wrt_axis_angle(Jacobian, axis, input);}
};

TEST_F(JacobianRotatedAxisTest, TestZeroInput) {
    input = Vector3f::Zero();
    calc_Jacobian();
    for (float x=- - 3; x < 3.1; x += 1) {
        for (float y = -3; y < 3.1; y += 1) {
            for (float z = -3; z < 3.1; z += 1) {
                axis = Vector3f{{x, y, z}};
                Jac_rotated_wrt_axis_angle(Jacobian, axis, input); //calc_Jacobian();
                EXPECT_LT(Jacobian.norm(), 0.001);
            }
        }
    }
}

TEST_F(JacobianRotatedAxisTest, TestZeroAxis) {
    axis = Vector3f::Zero();
    calc_Jacobian();
    for (float x=- - 3; x < 3.1; x += 1) {
        for (float y = -3; y < 3.1; y += 1) {
            for (float z = -3; z < 3.1; z += 1) {
                input = Vector3f{{x, y, z}};
                Jac_rotated_wrt_axis_angle(Jacobian, axis, input); //calc_Jacobian();
                ASSERT_LT(Jacobian.norm(), 0.001);
            }
        }
    }
}


TEST(CoordinatesTest, IdsxAreDifferent) {
    ASSERT_NE(LEFT_IDX, UP_IDX);
    ASSERT_NE(FORWARD_IDX, UP_IDX);
    ASSERT_NE(FORWARD_IDX, LEFT_IDX);
}

TEST(CoordinatesTest, IdxsMatchDirVectors) {
    Vector3f dir_vector;
    for (int i=0; i < 3; i++) {
        switch (i) {
            case LEFT_IDX:
                dir_vector = LEFT_DIR;  break;
            case FORWARD_IDX:
                dir_vector = FORWARD_DIR;   break;
            case UP_IDX:
                dir_vector = UP_DIR;    break;
        }
        ASSERT_EQ(1.0, dir_vector[i]);
        ASSERT_EQ(0.0, dir_vector[(i+1)%3]);
        ASSERT_EQ(0.0, dir_vector[(i+2)%3]);
    }
}

TEST(CoordinatesTest, RightLeftOpposite) {
    ASSERT_EQ(-LEFT_DIR[LEFT_IDX], RIGHT_DIR[LEFT_IDX]);
    Vector3f test_vec{2, 4, 5};
    Vector3f mirror_vec = RIGHTSCALE * test_vec.array();
    ASSERT_EQ(-test_vec[LEFT_IDX], mirror_vec[LEFT_IDX]);
    ASSERT_EQ(test_vec[FORWARD_IDX], mirror_vec[FORWARD_IDX]);
    ASSERT_EQ(test_vec[UP_IDX], mirror_vec[UP_IDX]);
}

