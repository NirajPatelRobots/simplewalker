#include "motor_model.hpp"


bool FcnChain::set_function(unsigned fcn_idx, ModelFcn *new_fcn) {
    if (fcn_idx > fcns.size()) {
        delete new_fcn;
        return false;
    }
    if (fcn_idx == fcns.size()) {
        fcns.emplace_back(new_fcn);
    } else {
        delete fcns[fcn_idx];
        fcns[fcn_idx] = new_fcn;
    }
    return true;
}


FcnChain::FcnChain(FcnChain &&other) noexcept
    : fcns(other.fcns), weight(other.weight) {
    other.fcns = {};
}

FcnChain &FcnChain::operator=(FcnChain &&other) noexcept {
    if (this != &other) {
        fcns = other.fcns;
        weight = other.weight;
        other.fcns = {};
    }
    return *this;
}


float FcnChain::call(const model_inputs_t &model_inputs) const {
    float return_val; // not used by base fcn, which is the first fcn called
    for (const auto &fcn : fcns) {
        return_val = fcn->call(model_inputs, return_val);
    }
    return weight * return_val;
}


// pass ownership of new_fcn to FcnChain, or delete it if it couldn't be added
bool LinearGroup::set_function(unsigned term_idx, unsigned fcn_idx, ModelFcn *new_fcn) {
    if (term_idx > terms.size() || (term_idx == terms.size() && fcn_idx != 0)) { 
        delete new_fcn;
        return false;
    }
    if (term_idx == terms.size()) {
        terms.push_back(new_fcn);
        return true;
    }
    return terms[term_idx].set_function(fcn_idx, new_fcn);
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


MotorModel::MotorModel() {
    state_terms.set_function(0, 0, ModelFcns::Vel::create());
    input_terms.set_function(0, 0, ModelFcns::One::create());
    state_terms.set_weight(0, -1.0);
}


float MotorModel::predict_accel(const model_inputs_t &model_inputs, float V) const {
    return state_terms.call(model_inputs) + V * input_terms.call(model_inputs);
}

float MotorModel::choose_V(const model_inputs_t &model_inputs, float accel) const {
    return (accel - state_terms.call(model_inputs)) / input_terms.call(model_inputs);
}
