#include <gtest/gtest.h>

#include "../../microcontroller/control/model_fcns.hpp"
#include "../../microcontroller/control/motor_model_API.hpp"


const float TOLERANCE = 1e-5;
const model_inputs_t INPUT_EXAMPLE = {.velocity = 104.3};
const float VOLTAGE_EX_0 = INPUT_EXAMPLE.velocity;
const float VOLTAGE_EX_1 = VOLTAGE_EX_0 + 1.0;
const int BASE_IDX_0 = 0;
const MotorModelAPI::GROUP bad_group = (MotorModelAPI::GROUP)(3);

const int8_t SUCCESS = MotorModelAPI::Response::SUCCESS;
const MotorModelAPI::GROUP GROUP_INPUT = MotorModelAPI::GROUP::INPUT;
const MotorModelAPI::GROUP GROUP_STATE = MotorModelAPI::GROUP::STATE;
const MotorModelAPI::COMMAND HELLO = MotorModelAPI::COMMAND::HELLO;


TEST(BadMessageCreationTest, BadGroup) {
    MotorModelAPI::Request msg_out{};

    EXPECT_EQ(msg_out.command, HELLO); // default message should be no-op
    EXPECT_FALSE(MotorModelAPI::msg_set_function(bad_group, 0, 0, "vel", msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_set_parameter(bad_group, 0, 0, 0, 1.0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_set_weight(bad_group, 0, 1.0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
}


TEST(BadMessageCreationTest, MismatchedBaseNonbaseFunction) {
    MotorModelAPI::Request msg_out{};
    for (unsigned term_idx = 0; term_idx < 3; term_idx++) {
        EXPECT_FALSE(MotorModelAPI::msg_set_function(GROUP_STATE, term_idx, BASE_IDX_0, "mult", msg_out));
        EXPECT_EQ(msg_out.command, HELLO);
        EXPECT_FALSE(MotorModelAPI::msg_set_function(GROUP_STATE, term_idx, 1, "vel", msg_out));
        EXPECT_EQ(msg_out.command, HELLO);
    }
}



class MotorModelAPITest : public testing::Test {
    MotorModelAPI::Response response;
protected:
    MotorModel model{};
    MotorModelAPI::Request request{};
    void do_api_call(MotorModelAPI::Response::STATUS expected_status = MotorModelAPI::Response::SUCCESS) {
        response = {.status = MotorModelAPI::Response::UNSET, .group = bad_group, .term_idx = UINT8_MAX, .fcn_idx = UINT8_MAX};
        MotorModelAPI::handle_request(request, model, response);
        EXPECT_EQ(response.status, expected_status);
        EXPECT_EQ(response.group, request.group);
        EXPECT_EQ(response.term_idx, request.term_idx);
        EXPECT_EQ(response.fcn_idx, request.fcn_idx);
    }
    void expect_default_model() {
        EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), VOLTAGE_EX_0, TOLERANCE);
        EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 1.0), VOLTAGE_EX_1, TOLERANCE);
    }
};


TEST_F(MotorModelAPITest, Hello) {
    request = {};
    do_api_call(MotorModelAPI::Response::HELLO);
    request = MotorModelAPI::HELLO_REQUEST;  // alias for default
    do_api_call(MotorModelAPI::Response::HELLO);
    expect_default_model();
}


TEST_F(MotorModelAPITest, CreateTerm) {
    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 1, BASE_IDX_0, "vel", request));
    do_api_call();
    // accel = vel - vel + V
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 2.0), 2.0, TOLERANCE);
    EXPECT_TRUE(MotorModelAPI::msg_set_weight(GROUP_STATE, 1, -1.0, request));
    do_api_call();
    // accel = vel + vel + V
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), 2* VOLTAGE_EX_0, TOLERANCE);
    EXPECT_EQ(model.state_terms.terms.size(), 2);

    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_INPUT, 1, BASE_IDX_0, "one", request));
    do_api_call();
    // accel = vel + vel + (1 + 1) * V
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), VOLTAGE_EX_0, TOLERANCE);
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 2.0), VOLTAGE_EX_1, TOLERANCE);
    EXPECT_EQ(model.input_terms.terms.size(), 2);
}


TEST_F(MotorModelAPITest, AddFunctionSetParam) {
    auto bad_param_response = MotorModelAPI::Response::BAD_PARAM_IDX;

    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 0, 1, "mult", request));
    do_api_call();
    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_INPUT, 0, 1, "mult", request));
    do_api_call();
    expect_default_model();

    // try to set param of Vel, which fails
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_STATE, 0, BASE_IDX_0, 0, 3.0, request));
    do_api_call(bad_param_response);
    // Try to set nonexistent param of mult, which fails
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_STATE, 0, 1, 1, 3.0, request));
    do_api_call(bad_param_response);
    expect_default_model();
    
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_STATE, 0, 1, 0, 3.0, request));
    do_api_call();
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), 3* VOLTAGE_EX_0, TOLERANCE);
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_INPUT, 0, 1, 0, 3.0, request));
    do_api_call();
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), VOLTAGE_EX_0, TOLERANCE);
}


TEST_F(MotorModelAPITest, ReplaceTerm) {
    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 0, 1, "mult", request));
    do_api_call();
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_STATE, 0, 1, 0, 3.0, request));
    do_api_call();
    // model: acc = V - 3 * vel
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), 3* VOLTAGE_EX_0, TOLERANCE);
    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 0, 0, "one", request));
    do_api_call();
    // replace vel with const 1, acc = V - 3
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), 3, TOLERANCE);
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 1.0), 4, TOLERANCE);
    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 0, 1, "mult", request));
    do_api_call();
    // replace mult with itself, which resets its parameter to 1, acc = V - 1
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), 1, TOLERANCE);
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, -1.0), 0, TOLERANCE);
    EXPECT_EQ(model.state_terms.terms.size(), 1);
}


TEST_F(MotorModelAPITest, SetWeight) {
    EXPECT_TRUE(MotorModelAPI::msg_set_weight(GROUP_STATE, 0, 3.0, request));
    do_api_call();
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), -3 * VOLTAGE_EX_0, TOLERANCE);

    EXPECT_TRUE(MotorModelAPI::msg_set_weight(GROUP_INPUT, 0, 3.0, request));
    do_api_call();
    EXPECT_NEAR(model.choose_V(INPUT_EXAMPLE, 0.0), -VOLTAGE_EX_0, TOLERANCE);
}

// BAD_REQUEST and BAD_FCN_ID aren't tested because you shouldn't be able to make bad requests or bad fcn ids


TEST_F(MotorModelAPITest, BadTermIndex) {
    expect_default_model();
    auto expected_response = MotorModelAPI::Response::BAD_TERM_IDX;
    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 2, BASE_IDX_0, "vel", request));
    do_api_call(expected_response);
    expect_default_model();

    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 2, 1, "mult", request));
    do_api_call(expected_response);
    expect_default_model();

    // this is also a bad param error but bad term idx should come first
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_STATE, 1, 0, 0, 1.0, request));
    do_api_call(expected_response);
    expect_default_model();

    EXPECT_TRUE(MotorModelAPI::msg_set_weight(GROUP_STATE, 1, 1.0, request));
    do_api_call(expected_response);
    expect_default_model();
}


TEST_F(MotorModelAPITest, BadFcnIndex) {
    auto expected_response = MotorModelAPI::Response::BAD_FCN_IDX;

    EXPECT_TRUE(MotorModelAPI::msg_set_function(GROUP_STATE, 0, 2, "mult", request));
    do_api_call(expected_response);
    expect_default_model();
    
    EXPECT_TRUE(MotorModelAPI::msg_set_parameter(GROUP_STATE, 0, 1, 0, 3.0, request));
    do_api_call(MotorModelAPI::Response::BAD_PARAM_IDX);  // slightly incorrect error message for convenience
    expect_default_model();
}
