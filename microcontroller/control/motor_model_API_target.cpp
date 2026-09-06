#include "model_fcns.hpp"
#include "motor_model_API.hpp"

namespace MotorModelAPI {
void handle_request(const Request &msg, MotorModel &model, Response &response) {
    LinearGroup *group = nullptr;
    response = {.status = response.UNSET, .term_idx = msg.term_idx};  // default
    response.term_idx = msg.term_idx;

    switch (msg.group)
    {
    case GROUP::STATE:
        group = &model.state_terms;
        break;
    case GROUP::INPUT:
        group = &model.input_terms;
        break;
    default:
        response = {.status = response.BAD_REQUEST, .term_idx = 0};
        return;
    }

    if (group->terms.empty() && msg.command != COMMAND::CREATE_TERM) {
        response = {.status = response.BAD_TERM_IDX, .term_idx = 0};
        return;
    }

    switch (msg.command)
    {
    case COMMAND::CREATE_TERM:
        if (msg.fcn_field >= NUM_BASE_FUNCTIONS) {
            response = {.status = response.BAD_FCN_FIELD, .term_idx = 0};
            return;
        }
        group->create_term(BASE_FUNCTIONS[msg.fcn_field](), msg.value);
        response.term_idx = group->terms.size();
        break;
    case COMMAND::DELETE_TERM:
        break;
    case COMMAND::ADD_FCN:
        break;
    case COMMAND::SET_PARAM:
        break;
    case COMMAND::SET_WEIGHT:
        break;
    default:
        response = {.status = response.BAD_REQUEST, .term_idx = 0};
        break;
    }
    response.status = response.SUCCESS;
}
}
