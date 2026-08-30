#include <gtest/gtest.h>

#include "../../microcontroller/control/motor_model.hpp"

const float TOLERANCE = 1e-6;
const model_inputs_t INPUT_EXAMPLE = {.velocity = 104.3};


class MockModelFcn : public ModelFcn {
    MockModelFcn() : ModelFcn(nullptr) {
        num_instances++;
    };
public:
    static ModelFcn *create() {return new MockModelFcn;}
    virtual ~MockModelFcn() {
        num_instances--;
    }
    inline float call(const model_inputs_t &model_inputs) const override {
        return parent->call(model_inputs);
    }
    static int num_instances;
};
int MockModelFcn::num_instances = 0 ;


// Tested: sizeof(ModelFcns::x) / sizeof(char) = 16 byte
TEST(FcnsTest, VelModelFcn) {
    auto vel_fcn = ModelFcns::Vel::create();

    EXPECT_NEAR(INPUT_EXAMPLE.velocity, vel_fcn->call(INPUT_EXAMPLE), TOLERANCE);
    EXPECT_FALSE(vel_fcn->set_param(0, 0.f));
}

TEST(FcnsTest, OneModelFcn) {
    auto one_fcn = ModelFcns::One::create();

    EXPECT_EQ(1.f, one_fcn->call(INPUT_EXAMPLE));
    EXPECT_FALSE(one_fcn->set_param(0, 0.f));
}

TEST(FcnsTest, SignModelFcn) {
    auto sign_fcn = ModelFcns::Sign::create();
    sign_fcn->set_parent(ModelFcns::Vel::create());

    model_inputs_t inputs = {.velocity = 100.f};
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
    EXPECT_NEAR(weight * INPUT_EXAMPLE.velocity, term.call(INPUT_EXAMPLE), TOLERANCE);
}

TEST(FcnChainTest, OnlyVel) {
    FcnChain term = FcnChain(ModelFcns::Vel::create(), 1.0);

    EXPECT_NEAR(INPUT_EXAMPLE.velocity, term.call(INPUT_EXAMPLE), TOLERANCE);
}

TEST(FcnChainTest, SignVel) {
    auto term = FcnChain(ModelFcns::Vel::create(), 1.0);
    term.add_function(ModelFcns::Sign::create());

    model_inputs_t inputs = {.velocity = 100.f};
    EXPECT_EQ( 1.f, term.call(inputs));  // intentional float equality comparisons
    inputs.velocity = -100.f;
    EXPECT_EQ(-1.f, term.call(inputs));
    inputs.velocity = 0.f;
    EXPECT_EQ( 0.f, term.call(inputs));
}


TEST(FcnChainTest, ThreeDeep) {
    auto term = new FcnChain(ModelFcns::Vel::create(), 1.0);
    term->add_function(MockModelFcn::create());
    term->add_function(MockModelFcn::create());

    EXPECT_EQ(INPUT_EXAMPLE.velocity, term->call(INPUT_EXAMPLE));
    EXPECT_EQ(MockModelFcn::num_instances, 2);
    
    delete term;
    EXPECT_EQ(MockModelFcn::num_instances, 0);
}


TEST(FcnChainTest, TwoChainzSameBase) {
    auto term1 = new FcnChain(ModelFcns::Vel::create(), 1.0);
    term1->add_function(MockModelFcn::create());

    auto term2 = new FcnChain(ModelFcns::Vel::create(), 1.0);
    term2->add_function(MockModelFcn::create());

    EXPECT_EQ(MockModelFcn::num_instances, 2);
    EXPECT_EQ(term1->call(INPUT_EXAMPLE), INPUT_EXAMPLE.velocity);
    EXPECT_EQ(term2->call(INPUT_EXAMPLE), INPUT_EXAMPLE.velocity);
    delete term1;
    EXPECT_EQ(term2->call(INPUT_EXAMPLE), INPUT_EXAMPLE.velocity);
    EXPECT_EQ(MockModelFcn::num_instances, 1);
    delete term2;
    EXPECT_EQ(MockModelFcn::num_instances, 0);
}


TEST(FcnChainTest, TwoChainzDifferentBase) {
    auto termOne = new FcnChain(ModelFcns::One::create(), 1.0);
    termOne->add_function(MockModelFcn::create());
    
    auto termVel = new FcnChain(ModelFcns::Vel::create(), 1.0);
    termVel->add_function(MockModelFcn::create());
    
    model_inputs_t inputs = {.velocity = -100.f};

    EXPECT_EQ(MockModelFcn::num_instances, 2);
    EXPECT_EQ(termOne->call(inputs), 1.0);
    EXPECT_EQ(termVel->call(inputs), inputs.velocity);
    delete termOne;
    EXPECT_EQ(termVel->call(inputs), inputs.velocity);
    delete termVel;
    EXPECT_EQ(MockModelFcn::num_instances, 0);
}

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
