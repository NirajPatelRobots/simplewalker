#include "model_fcns.hpp"
#include "motor_model_API.hpp"
#include <array>
#include <string>
#include <unordered_map>


namespace MotorModelAPI {
// helpers
const uint8_t FCN_NOT_FOUND = UINT8_MAX;
std::unordered_map<std::string, ModelFcn *(*)(void)> BASE_FUNCTION_NAME_MAP {
    {"vel", ModelFcns::Vel::create},
    {"one", ModelFcns::One::create},
};

std::unordered_map<std::string, ModelFcn *(*)(void)> NONBASE_FUNCTION_NAME_MAP {
    {"mult", ModelFcns::Mult::create},
    {"sign", ModelFcns::Sign::create},
};

uint8_t get_base_fcn_id(const char *fcn_name) {
    const auto &creator_fcn = BASE_FUNCTION_NAME_MAP.find(fcn_name);
    if (creator_fcn == BASE_FUNCTION_NAME_MAP.end()) return FCN_NOT_FOUND;
    for (size_t i = 0; i < NUM_BASE_FUNCTIONS; i++) {
        if (BASE_FUNCTIONS[i] == creator_fcn->second) return i;
    }
    return FCN_NOT_FOUND;
}

uint8_t get_nonbase_fcn_id(const char *fcn_name) {
    const auto &creator_fcn = NONBASE_FUNCTION_NAME_MAP.find(fcn_name);
    if (creator_fcn == NONBASE_FUNCTION_NAME_MAP.end()) return FCN_NOT_FOUND;
    for (size_t i = 0; i < NUM_NONBASE_FUNCTIONS; i++) {
        if (NONBASE_FUNCTIONS[i] == creator_fcn->second) return i;
    }
    return FCN_NOT_FOUND;
}


bool msg_set_function(GROUP group, uint8_t term_idx, uint8_t fcn_idx, const char *fcn_name, Request &msg_out) {
    msg_out.group = group;
    msg_out.term_idx = term_idx;
    msg_out.fcn_idx = fcn_idx;
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; msg_out.group = GROUP::STATE; return false;}
    if (fcn_idx == 0) {
        msg_out.fcn_field = get_base_fcn_id(fcn_name);
    } else {
        msg_out.fcn_field = get_nonbase_fcn_id(fcn_name);
    }
    if (msg_out.fcn_field == FCN_NOT_FOUND) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.command = COMMAND::SET_FCN;
    return true;
}


bool msg_set_parameter(GROUP group, unsigned term_idx, unsigned fcn_idx, unsigned param_idx, float value, Request &msg_out) {
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.group = group;
    msg_out.command = COMMAND::SET_PARAM;
    msg_out.term_idx = term_idx;
    msg_out.fcn_idx = fcn_idx;
    msg_out.fcn_field = param_idx;
    msg_out.value = value;
    return true;
}


bool msg_set_weight(GROUP group, unsigned term_idx, float weight, Request &msg_out) {
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.group = group;
    msg_out.command = COMMAND::SET_WEIGHT;
    msg_out.term_idx = term_idx;
    msg_out.value = weight;
    return true;
}


constexpr size_t NUM_SUCCESS_STATUSES = 3;
constexpr size_t NUM_FAILURE_STATUSES = 6;
constexpr std::array<const char *, NUM_SUCCESS_STATUSES> success_status_names{"UNSET", "SUCCESS", "HELLO"};
constexpr std::array<const char *, NUM_FAILURE_STATUSES> failure_status_names{
    "UNSET", "BAD_REQUEST", "BAD_TERM_IDX", "BAD_FCN_IDX", "BAD_FCN_ID", "BAD_PARAM_IDX",
};
constexpr std::array<const char *, 4> request_command_names{
    "HELLO", "SET_FCN", "SET_PARAM", "SET_WEIGHT"
};

const char * response_status_name(Response::STATUS status) {
    if (status >= 0) {
        return success_status_names[status % NUM_SUCCESS_STATUSES];
    } else {
        return failure_status_names[-status % NUM_FAILURE_STATUSES];
    }
}

const char * request_command_name(COMMAND cmd) {
    return request_command_names[(uint8_t)cmd];
}
}
