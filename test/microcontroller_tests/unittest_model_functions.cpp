#include <gtest/gtest.h>

#include "../../microcontroller/control/model_fcns.hpp"

const float TOLERANCE = 1e-6;
const model_inputs_t INPUT_EXAMPLE = {.velocity = 104.3};
const float DUMMYVAL = 1234.56;


// Tested: sizeof(ModelFcns::VelModelFcn) / sizeof(char) = 16 byte
TEST(FcnsTest, Vel) {
    auto vel_fcn = ModelFcns::Vel::create();
    
    EXPECT_NEAR(INPUT_EXAMPLE.velocity, vel_fcn->call(INPUT_EXAMPLE, DUMMYVAL), TOLERANCE);
    EXPECT_FALSE(vel_fcn->set_param(0, 0.f));
    delete vel_fcn;
}

TEST(FcnsTest, One) {
    auto one_fcn = ModelFcns::One::create();
    
    EXPECT_EQ(1.f, one_fcn->call(INPUT_EXAMPLE, DUMMYVAL));
    EXPECT_FALSE(one_fcn->set_param(0, 0.f));
    delete one_fcn;
}

TEST(FcnsTest, Sign) {
    auto sign_fcn = ModelFcns::Sign::create();
    
    model_inputs_t inputs = {.velocity = DUMMYVAL};
    float parent_val = 100.f;
    EXPECT_FALSE(sign_fcn->set_param(0, 0.f));
    EXPECT_EQ( 1.f, sign_fcn->call(inputs, parent_val));  // intentional float equality comparisons
    parent_val = -100.f;
    EXPECT_EQ(-1.f, sign_fcn->call(inputs, parent_val));
    parent_val = 0.f;
    EXPECT_EQ( 0.f, sign_fcn->call(inputs, parent_val));
    delete sign_fcn;
}
