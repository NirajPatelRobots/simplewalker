#include <gtest/gtest.h>

#include "../../microcontroller/control/motor_model.hpp"

const float TOLERANCE = 1e-6;

TEST(ModelFcnTest, VelocityModelFcnTest) {
    VelocityModelFcn vel_fcn = VelocityModelFcn::getInstance();
    model_inputs_t inputs = {.velocity = 104.3};

    EXPECT_NEAR(inputs.velocity, vel_fcn.call(inputs), TOLERANCE);
}
