#include "model_fcns.hpp"
#include "motor_model_API.hpp"
#include <unordered_map>


namespace MotorModelAPI {
// helpers
std::unordered_map<std::string, ModelFcn *(*)(void)> BASE_FUNCTION_NAME_MAP {
    {"vel", ModelFcns::Vel::create},
    {"one", ModelFcns::One::create},
};

std::unordered_map<std::string, ModelFcn *(*)(void)> NONBASE_FUNCTION_NAME_MAP {
    {"mult", ModelFcns::Mult::create},
    {"sign", ModelFcns::Sign::create},
};

uint8_t get_base_fcn_id(const char *fcn_name) {
    const auto &creator_fcn = BASE_FUNCTION_NAME_MAP[fcn_name];
    for (size_t i = 0; i < NUM_BASE_FUNCTIONS; i++) {
        if (BASE_FUNCTIONS[i] == creator_fcn) return i;
    }
    return UINT8_MAX;
}

uint8_t get_nonbase_fcn_id(const char *fcn_name) {
    const auto &creator_fcn = NONBASE_FUNCTION_NAME_MAP[fcn_name];
    for (size_t i = 0; i < NUM_NONBASE_FUNCTIONS; i++) {
        if (NONBASE_FUNCTIONS[i] == creator_fcn) return i;
    }
    return UINT8_MAX;
}


// message creation functions
bool msg_create_term(GROUP group, const char *base_fcn_name, float weight, Request &msg_out) {
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.fcn_field = get_base_fcn_id(base_fcn_name);
    if (msg_out.fcn_field > NUM_BASE_FUNCTIONS) {
        msg_out.command = COMMAND::HELLO;
        return false;
    }
    msg_out.group = group;
    msg_out.command = COMMAND::CREATE_TERM;
    msg_out.value = weight;
    return true;
}


bool msg_delete_term(GROUP group, unsigned term_idx, Request &msg_out) {
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.group = group;
    msg_out.command = COMMAND::DELETE_TERM;
    msg_out.term_idx = term_idx;
    return true;
}


bool msg_add_function(GROUP group, unsigned term_idx, const char *nonbase_fcn_name, Request &msg_out) {
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.fcn_field = get_nonbase_fcn_id(nonbase_fcn_name);
    if (msg_out.fcn_field > NUM_NONBASE_FUNCTIONS) {
        msg_out.command = COMMAND::HELLO;
        return false;
    }
    msg_out.group = group;
    msg_out.command = COMMAND::ADD_FCN;
    msg_out.term_idx = term_idx;
    return true;
}


bool msg_set_parameter(GROUP group, unsigned term_idx, unsigned fcn_idx, unsigned param_idx, float value, Request &msg_out) {
    if (group > GROUP::INPUT) {msg_out.command = COMMAND::HELLO; return false;}
    if (fcn_idx > MAX_FCN_IDX) {msg_out.command = COMMAND::HELLO; return false;}
    if (param_idx > MAX_PARAM_IDX) {msg_out.command = COMMAND::HELLO; return false;}
    msg_out.group = group;
    msg_out.command = COMMAND::SET_PARAM;
    msg_out.term_idx = term_idx;
    msg_out.fcn_field = ((fcn_idx & 0xF) << 4) + (param_idx & 0xF);
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
}
