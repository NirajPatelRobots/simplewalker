#include "motor_model.hpp"


float MotorModel::predict(const model_inputs_t &model_inputs)  {
    float out = 0;
    for (const auto &fcn_chain : functions) {
        out += fcn_chain.call(model_inputs);
    }
    return out;
}
