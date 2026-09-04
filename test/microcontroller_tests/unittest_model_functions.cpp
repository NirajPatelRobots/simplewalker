#include <gtest/gtest.h>

#include "../../microcontroller/control/model_fcns.hpp"

const float TOLERANCE = 1e-6;
const model_inputs_t INPUT_EXAMPLE = {.velocity = 104.3};


// Tested: sizeof(ModelFcns::VelModelFcn) / sizeof(char) = 16 byte
TEST(FcnsTest, Vel) {
    auto vel_fcn = ModelFcns::Vel::create();
    
    EXPECT_NEAR(INPUT_EXAMPLE.velocity, vel_fcn->call(INPUT_EXAMPLE), TOLERANCE);
    EXPECT_FALSE(vel_fcn->set_param(0, 0.f));
    delete vel_fcn;
}

TEST(FcnsTest, One) {
    auto one_fcn = ModelFcns::One::create();
    
    EXPECT_EQ(1.f, one_fcn->call(INPUT_EXAMPLE));
    EXPECT_FALSE(one_fcn->set_param(0, 0.f));
    delete one_fcn;
}

TEST(FcnsTest, Sign) {
    auto sign_fcn = ModelFcns::Sign::create();
    sign_fcn->set_parent(ModelFcns::Vel::create());
    
    model_inputs_t inputs = {.velocity = 100.f};
    EXPECT_FALSE(sign_fcn->set_param(0, 0.f));
    EXPECT_EQ( 1.f, sign_fcn->call(inputs));  // intentional float equality comparisons
    inputs.velocity = -100.f;
    EXPECT_EQ(-1.f, sign_fcn->call(inputs));
    inputs.velocity = 0.f;
    EXPECT_EQ( 0.f, sign_fcn->call(inputs));
    delete sign_fcn;
}

TEST(FcnsTest, Mult) {
    auto mult_fcn = ModelFcns::Mult::create();
    mult_fcn->set_parent(ModelFcns::Vel::create());

    EXPECT_NEAR(mult_fcn->call(INPUT_EXAMPLE), INPUT_EXAMPLE.velocity, TOLERANCE);
    EXPECT_FALSE(mult_fcn->set_param(1, 2.f)); // no effect
    EXPECT_NEAR(mult_fcn->call(INPUT_EXAMPLE), INPUT_EXAMPLE.velocity, TOLERANCE);
    EXPECT_TRUE(mult_fcn->set_param(0, 2.f));
    EXPECT_NEAR(mult_fcn->call(INPUT_EXAMPLE), 2 * INPUT_EXAMPLE.velocity, TOLERANCE);
    delete mult_fcn;
}
