#include "motor_model.hpp"


bool ModelFcn::set_parent(ModelFcn *new_parent) {
    if (!parent) parent = new_parent;
    return parent == new_parent;
}

bool ModelFcn::set_param_recurse(unsigned fcn_idx, unsigned param_num, float value) {
    if (fcn_idx == 0) {
        return set_param(param_num, value);
    } else {
        return parent && parent->set_param_recurse(fcn_idx - 1, param_num, value);
    }
}


FcnChain::FcnChain(FcnChain &&other) noexcept
    : first_fcn(other.first_fcn), weight(other.weight) {
    other.first_fcn = nullptr;
}

FcnChain &FcnChain::operator=(FcnChain &&other) noexcept {
    if (this != &other) {
        delete first_fcn;
        first_fcn = other.first_fcn;
        weight = other.weight;
        other.first_fcn = nullptr;
    }
    return *this;
}

bool FcnChain::add_function(ModelFcn *new_fcn) {
    if (new_fcn->set_parent(first_fcn)) first_fcn = new_fcn;
    return first_fcn == new_fcn;
}


bool LinearGroup::set_weight(unsigned int term_idx, float weight) {
    if (term_idx < terms.size()) {
        terms[term_idx].weight = weight;
        return true;
    }
    return false;
}

float LinearGroup::call(const model_inputs_t &model_inputs) const {
    float out = 0;
    for (const auto &fcn_chain : terms) {
        out += fcn_chain.call(model_inputs);
    }
    return out;
}


float MotorModel::predict_accel(const model_inputs_t &model_inputs, float V) const {
    return state_terms.call(model_inputs) + V * input_terms.call(model_inputs);
}

float MotorModel::choose_V(const model_inputs_t &model_inputs, float accel) const {
    return (accel - state_terms.call(model_inputs)) / input_terms.call(model_inputs);
}
