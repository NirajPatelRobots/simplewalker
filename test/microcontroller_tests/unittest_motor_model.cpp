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
    delete vel_fcn;
}

TEST(FcnsTest, OneModelFcn) {
    auto one_fcn = ModelFcns::One::create();

    EXPECT_EQ(1.f, one_fcn->call(INPUT_EXAMPLE));
    EXPECT_FALSE(one_fcn->set_param(0, 0.f));
    delete one_fcn;
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
    delete sign_fcn;
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
    // TODO replace with different fcn (param mult)?
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

TEST(FcnChainTest, MoveObject) {
    float weight = 0.5;
    auto term = FcnChain(ModelFcns::Vel::create(), weight);
    term.add_function(MockModelFcn::create());

    FcnChain moveConstructed = std::move(term);

    EXPECT_EQ(weight, moveConstructed.weight);
    EXPECT_NEAR(weight * INPUT_EXAMPLE.velocity, moveConstructed.call(INPUT_EXAMPLE), TOLERANCE);
    
    FcnChain moveAssigned(ModelFcns::One::create(), 1.0);
    moveAssigned = std::move(moveConstructed);

    EXPECT_EQ(weight, moveAssigned.weight);
    EXPECT_NEAR(weight * INPUT_EXAMPLE.velocity, moveAssigned.call(INPUT_EXAMPLE), TOLERANCE);
}

// test FcnChain base function must be a type that doesn't reference a parent?


// LinearGroup
TEST(LinearGroupTest, CreateAddCallDelete) {
    auto group = LinearGroup();
    group.create_term(ModelFcns::Vel::create(), 1.0);
    group.add_function(0, MockModelFcn::create());
    
    EXPECT_EQ(group.call(INPUT_EXAMPLE), INPUT_EXAMPLE.velocity);
    EXPECT_EQ(MockModelFcn::num_instances, 1);
    group.delete_term(0);
    EXPECT_EQ(MockModelFcn::num_instances, 0);
}


TEST(LinearGroupTest, OnePlusOne) {
    auto group = LinearGroup();
    group.create_term(ModelFcns::One::create(), 1.0);
    group.create_term(ModelFcns::One::create(), 1.0);
    group.add_function(0, MockModelFcn::create());

    EXPECT_NEAR(group.call(INPUT_EXAMPLE), 2.0, TOLERANCE);
    EXPECT_EQ(MockModelFcn::num_instances, 1);
    group.delete_term(1);
    EXPECT_NEAR(group.call(INPUT_EXAMPLE), 1.0, TOLERANCE);
    EXPECT_EQ(MockModelFcn::num_instances, 1);
}

// Todo test set_param


// Motor Model
TEST(MotorModelTest, OnePlusOnePredictAccel) {
    auto model = MotorModel();
    model.state_terms.create_term(ModelFcns::One::create(), 1.0);
    model.input_terms.create_term(ModelFcns::One::create(), 1.0);

    EXPECT_NEAR(1., model.predict_accel(INPUT_EXAMPLE, 0.), TOLERANCE);
    EXPECT_NEAR(2., model.predict_accel(INPUT_EXAMPLE, 1.), TOLERANCE);
}

TEST(MotorModelTest, OnePlusOneChooseV) {
    auto model = MotorModel();
    model.state_terms.create_term(ModelFcns::One::create(), 1.0);
    model.input_terms.create_term(ModelFcns::One::create(), 1.0);

    EXPECT_NEAR(0., model.choose_V(INPUT_EXAMPLE, 1.), TOLERANCE);
    EXPECT_NEAR(1., model.choose_V(INPUT_EXAMPLE, 2.), TOLERANCE);
}
