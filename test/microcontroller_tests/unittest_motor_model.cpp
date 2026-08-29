#include <gtest/gtest.h>

#include "../../microcontroller/control/motor_model.hpp"

const float TOLERANCE = 1e-6;
model_inputs_t inputs = {.velocity = 104.3};


TEST(FcnsTest, VelModelFcn) {
    auto vel_fcn = ModelFcns::Vel::create();

    EXPECT_NEAR(inputs.velocity, vel_fcn->call(inputs), TOLERANCE);
    EXPECT_FALSE(vel_fcn->set_param(0, 0.f));
}

TEST(FcnsTest, OneModelFcn) {
    auto one_fcn = ModelFcns::One::create();

    EXPECT_EQ(1.f, one_fcn->call(inputs));
    EXPECT_FALSE(one_fcn->set_param(0, 0.f));
}

TEST(FcnsTest, SignModelFcn) {
    auto sign_fcn = ModelFcns::Sign::create();
    sign_fcn->set_parent(ModelFcns::Vel::create());

    inputs.velocity = 100.f;
    EXPECT_EQ( 1.f, sign_fcn->call(inputs));  // intentional float equality comparisons
    inputs.velocity = -100.f;
    EXPECT_EQ(-1.f, sign_fcn->call(inputs));
    inputs.velocity = 0.f;
    EXPECT_EQ( 0.f, sign_fcn->call(inputs));
}


// Fcn Chain
TEST(FcnChainTest, Weight) {
    float weight = 0.5;
    FcnChain term = FcnChain(ModelFcns::Vel::create(), weight);

    EXPECT_EQ(weight, term.weight);
    EXPECT_NEAR(weight * inputs.velocity, term.call(inputs), TOLERANCE);
}

TEST(FcnChainTest, OnlyVel) {
    FcnChain term = FcnChain(ModelFcns::Vel::create(), 1.0);

    EXPECT_NEAR(inputs.velocity, term.call(inputs), TOLERANCE);
}

TEST(FcnChainTest, SignVel) {
    auto term = FcnChain(ModelFcns::Vel::create(), 1.0);
    term.add_function(ModelFcns::Sign::create());

    inputs.velocity = 100.f;
    EXPECT_EQ( 1.f, term.call(inputs));  // intentional float equality comparisons
    inputs.velocity = -100.f;
    EXPECT_EQ(-1.f, term.call(inputs));
    inputs.velocity = 0.f;
    EXPECT_EQ( 0.f, term.call(inputs));
}

// Test that modelFcns get deleted when FcnChain gets deleted, but singletons dont

// test FcnChain last function (first entered) must be a type that doesn't reference a parent?


// Motor Model
TEST(MotorModelTest, CreateAddToChain) {
    auto model = MotorModel();
    for (auto *terms : {&model.state_terms, &model.input_terms}) {
        terms->emplace_back(ModelFcns::Vel::create(), 1.0);
    }
}

TEST(MotorModelTest, AddToChain) {
    auto model = MotorModel();
    for (auto *terms : {&model.state_terms, &model.input_terms}) {
        terms->emplace_back(ModelFcns::Vel::create(), 1.0);
        terms->at(0).add_function(ModelFcns::Sign::create());
    }
}

// predict_accel, choose_V
// Create two chains, delete one and check the other still works
