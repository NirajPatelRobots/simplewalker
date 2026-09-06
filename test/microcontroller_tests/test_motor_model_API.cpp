#include <gtest/gtest.h>

#include "../../microcontroller/control/model_fcns.hpp"
#include "../../microcontroller/control/motor_model_API.hpp"


const float TOLERANCE = 1e-6;
const model_inputs_t INPUT_EXAMPLE = {.velocity = 104.3};
const float VOLTAGE_EX_0 = -INPUT_EXAMPLE.velocity;
const float VOLTAGE_EX_1 = VOLTAGE_EX_0 + 1.0;

const int8_t SUCCESS = MotorModelAPI::Response::SUCCESS;
const MotorModelAPI::GROUP GROUP_INPUT = MotorModelAPI::GROUP::INPUT;
const MotorModelAPI::GROUP GROUP_STATE = MotorModelAPI::GROUP::STATE;
const MotorModelAPI::COMMAND HELLO = MotorModelAPI::COMMAND::HELLO;


TEST(MessageCreationTest, BadGroup) {
    MotorModelAPI::GROUP bad_group = (MotorModelAPI::GROUP)(3);
    MotorModelAPI::Request msg_out{};

    EXPECT_EQ(msg_out.command, HELLO); // default message should be no-op
    EXPECT_FALSE(MotorModelAPI::msg_create_term(bad_group, "vel", 1.0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_delete_term(bad_group, 0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_add_function(bad_group, 0, "mult", msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_set_parameter(bad_group, 0, 0, 1.0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_set_weight(bad_group, 0, 1.0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
}


TEST(MessageCreationTest, MismatchedBaseNonbaseFunction) {
    MotorModelAPI::Request msg_out{};
    EXPECT_FALSE(MotorModelAPI::msg_create_term(GROUP_STATE, "mult", 1.0, msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
    EXPECT_FALSE(MotorModelAPI::msg_add_function(GROUP_STATE, 0, "vel", msg_out));
    EXPECT_EQ(msg_out.command, HELLO);
}


class MotorModelAPITest : public testing::Test {
    MotorModelAPI::Response response;
protected:
    MotorModel model{};
    MotorModelAPI::Request request{};
    void do_api_call(const MotorModelAPI::Response &expected_response) {
        response = {.status = MotorModelAPI::Response::UNSET, .term_idx = UINT8_MAX};
        MotorModelAPI::handle_request(request, model, response);
        std::cout << "Response status: " << (int)(response.status)
                  << ", idx:"            << (int)(response.term_idx) << "\n";
        EXPECT_EQ(response.status, expected_response.status);
        EXPECT_EQ(response.term_idx, expected_response.term_idx);
    }
    void expect_default_model() {
        EXPECT_EQ(model.choose_V(INPUT_EXAMPLE, 0.0), VOLTAGE_EX_0);
        EXPECT_EQ(model.choose_V(INPUT_EXAMPLE, 1.0), VOLTAGE_EX_1);
    }
};


TEST_F(MotorModelAPITest, Hello) {
    MotorModelAPI::Response expected_response = {
        .status = MotorModelAPI::Response::HELLO, 
        .term_idx = 0
    };
    request = {};
    do_api_call(expected_response);
}


TEST_F(MotorModelAPITest, CreateTerm) {
    MotorModelAPI::Response expected_response = {.status = SUCCESS, .term_idx = 0};

    MotorModelAPI::msg_create_term(GROUP_INPUT, "vel", 1.0, request);
    do_api_call(expected_response);
    model.choose_V(INPUT_EXAMPLE, 0.0);
}


TEST_F(MotorModelAPITest, DeleteVel) {
    MotorModelAPI::Response expected_response = {.status = SUCCESS, .term_idx = 0};

    EXPECT_EQ(model.state_terms.terms.size(), 1);
    EXPECT_EQ(model.input_terms.terms.size(), 1);
    // input
    EXPECT_TRUE(MotorModelAPI::msg_delete_term(GROUP_INPUT, 0, request));
    do_api_call(expected_response);
    EXPECT_EQ(model.state_terms.terms.size(), 0);
    // state
    EXPECT_TRUE(MotorModelAPI::msg_delete_term(GROUP_INPUT, 0, request));
    do_api_call(expected_response);
    EXPECT_EQ(model.input_terms.terms.size(), 0);
    // none left, all term indexes are bad term indexes
    expected_response = {.status = MotorModelAPI::Response::BAD_TERM_IDX, .term_idx = 0};
    EXPECT_TRUE(MotorModelAPI::msg_delete_term(GROUP_INPUT, 0, request));
    do_api_call(expected_response);
}


TEST_F(MotorModelAPITest, BadTermIndex) {
    expect_default_model();
    MotorModelAPI::Response expected_response = {
        .status = MotorModelAPI::Response::BAD_TERM_IDX, 
        .term_idx = 1
    };

    MotorModelAPI::msg_delete_term(GROUP_STATE, 1, request);
    do_api_call(expected_response);
    expect_default_model();
}


