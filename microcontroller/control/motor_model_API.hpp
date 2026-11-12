/* Common header for motor model API user and target.
    TODO:
        Separate TermRequest (create, delete, set weight) and FunctionRequest structs?
        user API imports model_fcns to get IDs. Separate them. Parse IDs to file? Common enum?
*/
#ifndef SIMPLEWALKER_PICO_MOTOR_MODEL_API_HPP
#define SIMPLEWALKER_PICO_MOTOR_MODEL_API_HPP
#include <stdint.h>

class MotorModel;


const uint16_t MotorModelAPIRequestID = 0x0F01;
const uint16_t MotorModelAPIResponseID = 0x0F02;

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
    uint16_t message_ID{MotorModelAPIRequestID};
    GROUP   group;
    COMMAND command;
    uint8_t term_idx;
    uint8_t fcn_field;
    float value;
};

struct Response {
    uint16_t message_ID{MotorModelAPIResponseID};
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
// Additional user functions
const Request HELLO_REQUEST {};
const char *response_status_name(Response::STATUS status);
const char *request_command_name(COMMAND cmd);

// Microcontroller API handler function
void handle_request(const Request &msg, MotorModel &model, Response &response);


}
#endif  // SIMPLEWALKER_PICO_MOTOR_MODEL_API_HPP
