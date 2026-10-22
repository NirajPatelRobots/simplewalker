#ifndef SIMPLEWALKER_PICO_MOTOR_MODEL_API_HPP
#define SIMPLEWALKER_PICO_MOTOR_MODEL_API_HPP
#include <stdint.h>

class MotorModel;

namespace MotorModelAPI {

enum class COMMAND : uint8_t {
    HELLO, // no effect, set if command creation fails
    CREATE_TERM,
    DELETE_TERM,
    ADD_FCN,
    SET_PARAM,
    SET_WEIGHT
};
enum class GROUP : uint8_t {
    STATE, INPUT
};
const uint8_t MAX_FCN_IDX{0xF};
const uint8_t MAX_PARAM_IDX{0xF};

struct Request {
    GROUP   group;
    COMMAND command;
    uint8_t term_idx;
    uint8_t fcn_field;
    float value;
};

struct Response {
    int8_t status;
    uint8_t term_idx;
    enum STATUS : int8_t {
        SUCCESS = 1, HELLO = 2,
        UNSET = 0, BAD_REQUEST = -1, BAD_TERM_IDX = -2, BAD_FCN_FIELD = -3, BAD_PARAM = -4,
    };
    inline bool successful() {return status > 0;}
};

// Request API functions (run on main computer)
bool msg_create_term(GROUP group, const char *base_fcn_name, float weight, Request &msg_out);
bool msg_delete_term(GROUP group, unsigned term_idx, Request &msg_out);
bool msg_add_function(GROUP group, unsigned term_idx, const char *nonbase_fcn_name, Request &msg_out);
bool msg_set_parameter(GROUP group, unsigned term_idx, unsigned fcn_idx, unsigned param_idx, float value, Request &msg_out);
bool msg_set_weight(GROUP group, unsigned term_idx, float weight, Request &msg_out);
const Request HELLO_REQUEST {};

// Microcontroller API handler function
void handle_request(const Request &msg, MotorModel &model, Response &response);

}
#endif  // SIMPLEWALKER_PICO_MOTOR_MODEL_API_HPP
