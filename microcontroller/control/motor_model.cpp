#include "motor_model.hpp"


bool ModelFcn::set_parent(ModelFcn *new_parent) {
    if (!parent) parent = new_parent;
    return parent == new_parent;
}



bool FcnChain::add_function(ModelFcn *new_fcn) {
    if (new_fcn->set_parent(first_fcn)) first_fcn = new_fcn;
    return first_fcn == new_fcn;
}


float MotorModel::predict_accel(const model_inputs_t &model_inputs, float V) {
    float out = 0;
    for (const auto &fcn_chain : state_terms) {
        out += fcn_chain.call(model_inputs);
    }
    for (const auto &fcn_chain : input_terms) {
        out += V * fcn_chain.call(model_inputs);
    }
    return out;
}

float MotorModel::choose_V(const model_inputs_t &model_inputs, float accel) {
    return 0;
}
