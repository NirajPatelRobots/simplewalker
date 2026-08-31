/* See README.md #"Motor Models"
 * TODO:
 *      FcnChain enforce initial base_fcn is a candidate base function (doesn't need parent)
 *      ModelFcns different data structure instead of subclass?
 *          Template<size_t num_params, float fcn(const model_inputs_t &model_inputs)>?
 *          std::pair<size_t, float fcn(const model_inputs_t &model_inputs)>?
 *      Programatically create function chains from a command API
 *      storage structure for modelFcns, index is ID
 */
#include <array>
#include <vector>

struct model_inputs_t {
    float velocity;
};

class ModelFcn {
protected:
    ModelFcn *parent;
public:
    ModelFcn(ModelFcn *_parent) : parent(_parent) {}
    virtual ~ModelFcn() {delete parent;}
    // set_parent and set_param return true if the parent/param was set
    bool set_parent(ModelFcn *new_parent);
    virtual bool set_param([[maybe_unused]] unsigned num, [[maybe_unused]] float value) {return false;}
    bool set_param(size_t fcn_idx, size_t param_num, float value);
    virtual float call(const model_inputs_t &model_inputs) const = 0;
};

// FcnChain owns the memory for its first_fcn, functions own their parent memory
struct FcnChain {
    FcnChain(ModelFcn *base_fcn, float _weight) : first_fcn(base_fcn), weight(_weight) {}
    FcnChain(FcnChain &&other) noexcept;  // movable but not copyable
    FcnChain& operator=(FcnChain&& other) noexcept;
    ~FcnChain() {delete first_fcn;}
    ModelFcn *first_fcn;
    float weight;
    bool add_function(ModelFcn *new_fcn);
    inline float call(const model_inputs_t &model_inputs) const {
        return weight * first_fcn->call(model_inputs);
    }
};

struct LinearGroup {
    std::vector<FcnChain> terms;
    inline void create_term(ModelFcn *base_fcn, float weight) {
        terms.emplace_back(base_fcn, weight);
    }
    inline void delete_term(unsigned term_idx) {
        terms.erase(terms.begin() + term_idx);
    }
    inline bool add_function(unsigned term_idx, ModelFcn *fcn) {
        return (term_idx < terms.size() && terms[term_idx].add_function(fcn));
    }
    inline bool set_param(unsigned term_idx, unsigned fcn_idx, unsigned param_num, float value) {
        return (term_idx < terms.size()
            && terms[term_idx].first_fcn->set_param(fcn_idx, param_num, value));
    }
    inline bool set_weight(unsigned term_idx, float weight);
    float call(const model_inputs_t &model_inputs) const;
};

class MotorModel {
public:
    LinearGroup state_terms;
    LinearGroup input_terms;
    float predict_accel(const model_inputs_t &model_inputs, float V) const;
    float choose_V(const model_inputs_t &model_inputs, float accel) const;
};

namespace ModelFcns {
    // base functions
    class Vel : public ModelFcn {
        Vel() : ModelFcn(nullptr) {}
    public:
        static ModelFcn *create() {return new Vel;}
        inline float call(const model_inputs_t &model_inputs) const override {
            return model_inputs.velocity;
        }
    };
    class One : public ModelFcn {
        One() : ModelFcn(nullptr) {}
    public:
        static ModelFcn *create() {return new One;}
        inline float call([[maybe_unused]] const model_inputs_t &_) const override {
            return 1;
        }
    };
    
    // non-base functions
    class Sign : public ModelFcn {
        Sign() : ModelFcn(nullptr) {};
    public:
        static ModelFcn *create() {return new Sign;}
        inline float call(const model_inputs_t &model_inputs) const override {
            float parentval = parent->call(model_inputs);
            return parentval == 0 ? 0.f : parentval > 0 ? 1.f : -1.f;
        }
    };
}  // namespace ModelFcns
