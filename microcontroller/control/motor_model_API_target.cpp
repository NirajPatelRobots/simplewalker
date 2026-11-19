#include "model_fcns.hpp"
#include "motor_model_API.hpp"

namespace MotorModelAPI {
Response::STATUS execute_request(LinearGroup *group, const Request &request);


void handle_request(const Request &request, MotorModel &model, Response &response) {
    LinearGroup *group = nullptr;
    response = {.status = response.UNSET, .group = request.group, .command = request.command,
                .term_idx = request.term_idx, .fcn_idx = request.fcn_idx, .fcn_field = request.fcn_field};

    switch (request.group)
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

    response.status = execute_request(group, request);
}

Response::STATUS execute_request(LinearGroup *group, const Request &request) {
    switch (request.command)
    {
    case COMMAND::HELLO:
        return Response::HELLO;
    case COMMAND::SET_FCN: {
            if (request.term_idx > group->terms.size()) {
                return Response::BAD_TERM_IDX;
            }
            ModelFcn *new_fcn;
            unsigned new_fcn_id = request.fcn_field;  // don't confuse this with fcn_idx
            if (request.fcn_idx == 0) {  // Base Fcn
                if (new_fcn_id >= (unsigned)BaseFunctionID::NUM_FCNS) {
                    return Response::BAD_FCN_ID;
                }
                new_fcn = BASE_FUNCTIONS[new_fcn_id]();
            } else {  // non-base
                if (new_fcn_id >= (unsigned)NonBaseFunctionID::NUM_FCNS) {
                    return Response::BAD_FCN_ID;
                }
                new_fcn = NONBASE_FUNCTIONS[new_fcn_id]();
            }
            if (!group->set_function(request.term_idx, request.fcn_idx, new_fcn)) {
                return Response::BAD_FCN_IDX;
            }
        }
        break;
    case COMMAND::SET_PARAM:
        if (request.term_idx >= group->terms.size()) {
            return Response::BAD_TERM_IDX;
        }
        // Here, request.fcn_field is param_idx. Don't confuse this with fcn_idx or fcn_ID.
        if (!group->set_param(request.term_idx, request.fcn_idx, request.fcn_field, request.value)) {
            return Response::BAD_PARAM_IDX;
        }
        break;
    case COMMAND::SET_WEIGHT:
        if (request.term_idx >= group->terms.size()) {
            return Response::BAD_TERM_IDX;
        }
        group->set_weight(request.term_idx, request.value);
        break;
    default:
        return Response::BAD_REQUEST;
    }
    return Response::SUCCESS;
}
}
