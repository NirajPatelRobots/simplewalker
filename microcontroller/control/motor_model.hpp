/* See README.md #"Motor Models"
 * TODO:
 *      Standardize ModelFcns params
 *          const int ModelFcn::n_params; set by subclass constructor? 
 *          float *ModelFcn::params; point to first param, equivalent to array of param?
 *              hack: Enforce first data of function past ModelFcn is function->param[0]?
 *          2D array[term_idx, fcn_idx] of optional parameters_t passed as arg to call(), ModelFcns have no data
 *      MotorModel check input terms is not zero? (Skill issue?)
 *      How to remove functions? Remove all above given fcn_idx. Don't allow delete base?
 *      Non-parameterized functions could be singletons?
 */
#ifndef SIMPLEWALKER_PICO_MOTOR_MODEL_HPP
#define SIMPLEWALKER_PICO_MOTOR_MODEL_HPP
#include <array>
#include <vector>

struct model_inputs_t {
    float velocity;
};

class ModelFcn {
public:
    // set_param return true if the param was set
    virtual bool set_param([[maybe_unused]] unsigned num, [[maybe_unused]] float value) {return false;}
    virtual float call(const model_inputs_t &model_inputs, float parent_val) const = 0;
    virtual ~ModelFcn() = default;
};

// FcnChain takes ownership of the memory of its functions
struct FcnChain {
    FcnChain(ModelFcn *base_fcn) : fcns(), weight(1.0) {
        fcns.emplace_back(base_fcn);
    }
    FcnChain(FcnChain &&other) noexcept;  // movable but not copyable
    FcnChain& operator=(FcnChain&& other) noexcept;
    ~FcnChain() {
        for (auto *const fcn : fcns) delete fcn;
    }
    std::vector<ModelFcn*> fcns;
    float weight;
    bool set_function(unsigned fcn_idx, ModelFcn *new_fcn);
    float call(const model_inputs_t &model_inputs) const;
};

struct LinearGroup {
    std::vector<FcnChain> terms;
    inline void delete_term(unsigned term_idx) {
        terms.erase(terms.begin() + term_idx);
    }
    bool set_function(unsigned term_idx, unsigned fcn_idx, ModelFcn *new_fcn);
    inline bool set_param(unsigned term_idx, unsigned fcn_idx, unsigned param_num, float value) {
        return (term_idx < terms.size() && fcn_idx < terms[term_idx].fcns.size()
            && terms[term_idx].fcns[fcn_idx]->set_param(param_num, value));
    }
    bool set_weight(unsigned term_idx, float weight);
    float call(const model_inputs_t &model_inputs) const;
};

class MotorModel {
public:
    MotorModel();
    LinearGroup state_terms;
    LinearGroup input_terms;
    float predict_accel(const model_inputs_t &model_inputs, float V) const;
    float choose_V(const model_inputs_t &model_inputs, float accel) const;
};

namespace ModelFcns {
    // base functions
    class Vel : public ModelFcn {
        Vel() : ModelFcn() {}
    public:
        static ModelFcn *create() {return new Vel;}
        inline float call(const model_inputs_t &model_inputs, [[maybe_unused]] float _) const override {
            return model_inputs.velocity;
        }
    };
    class One : public ModelFcn {
        One() : ModelFcn() {}
    public:
        static ModelFcn *create() {return new One;}
        inline float call([[maybe_unused]] const model_inputs_t &_, [[maybe_unused]] float __) const override {
            return 1;
        }
    };
}  // namespace ModelFcns

#endif  // SIMPLEWALKER_PICO_MOTOR_MODEL_HPP
