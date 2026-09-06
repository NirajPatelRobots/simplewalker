#include "model_fcns.hpp"
#include "motor_model_API.hpp"

namespace MotorModelAPI {
void handle_request(const Request &msg, MotorModel &model, Response &response) {
    LinearGroup *group = nullptr;
    response = {.status = response.UNSET, .term_idx = msg.term_idx};  // default

    switch (msg.group)
    {
    case GROUP::STATE:
        group = &model.state_terms;
        break;
    case GROUP::INPUT:
        group = &model.input_terms;
        break;
    default:
        response.status = response.BAD_REQUEST;
        return;
    }

    if (group->terms.size() <= msg.term_idx && msg.command > COMMAND::CREATE_TERM) {
        response.status = response.BAD_TERM_IDX;
        return;
    }

    switch (msg.command)
    {
    case COMMAND::HELLO:
        response.status = response.HELLO;
        return;
    case COMMAND::CREATE_TERM:
        if (msg.fcn_field >= NUM_BASE_FUNCTIONS) {
            response = {.status = response.BAD_FCN_FIELD, .term_idx = 0};
            return;
        }
        group->create_term(BASE_FUNCTIONS[msg.fcn_field](), msg.value);
        response.term_idx = group->terms.size();
        break;
    case COMMAND::DELETE_TERM:
        group->delete_term(msg.term_idx);
        response.term_idx = group->terms.size();
        break;
    case COMMAND::ADD_FCN:
        if (msg.fcn_field >= NUM_NONBASE_FUNCTIONS) {
            response = {.status = response.BAD_FCN_FIELD, .term_idx = 0};
            return;
        }
        group->add_function(msg.term_idx, NONBASE_FUNCTIONS[msg.fcn_field]());
        break;
    case COMMAND::SET_PARAM:
        {
            uint8_t fcn_idx = ((msg.fcn_field & 0xF0) >> 4);
            uint8_t param_idx = (msg.fcn_field & 0xF);
            if (!group->set_param(msg.term_idx, fcn_idx, param_idx, msg.value)) {
                response.status = response.BAD_PARAM;
                return;
            }
        }
        break;
    case COMMAND::SET_WEIGHT:
        group->set_weight(msg.term_idx, msg.value);
        break;
    default:
        response = {.status = response.BAD_REQUEST, .term_idx = 0};
        return;
    }
    response.status = response.SUCCESS;
}
}
