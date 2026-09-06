#include "motor_model_API.hpp"


namespace MotorModelAPI {
// helpers
uint8_t get_base_fcn_id(const char *fcn_name) {
    return 0;
}


uint8_t get_nonbase_fcn_id(const char *fcn_name) {
    return 0;
}


bool msg_create_term(GROUP group, const char *base_fcn_name, float weight, Request &msg_out) {
    return true;
}


bool msg_delete_term(GROUP group, unsigned term_idx, Request &msg_out) {
    return true;
}


bool msg_add_function(GROUP group, unsigned term_idx, const char *nonbase_fcn_name, Request &msg_out) {
    return true;
}


bool msg_set_parameter(GROUP group, unsigned term_idx, unsigned fcn_idx, unsigned param_idx, Request &msg_out) {
    return true;
}


bool msg_set_weight(GROUP group, unsigned term_idx, float weight, Request &msg_out) {
    return true;
}
}
