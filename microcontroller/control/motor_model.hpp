/* See README.md #"Motor Models"
 * TODO:
 *      Programatically create function chains from a command API
 *          storage structure for modelFcns, index is ID
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
    // set_parent and set_param return true if the parent/param was set
    bool set_parent(ModelFcn *new_parent);
    virtual bool set_param([[maybe_unused]] unsigned num, [[maybe_unused]] float value) {return false;}
    virtual float call(const model_inputs_t &model_inputs) const = 0;
    bool has_parent() {return (bool)parent;}
};

// FcnChain owns the memory for all of its ModelFcns except Singletons, which own themselves.
struct FcnChain {
    FcnChain(ModelFcn *base_fcn, float _weight) : first_fcn(base_fcn), weight(_weight) {}
    ModelFcn *first_fcn;
    float weight;
    bool add_function(ModelFcn *new_fcn);
    inline float call(const model_inputs_t &model_inputs) const {
        return weight * first_fcn->call(model_inputs);
    }
};

class MotorModel {
public:
    std::vector<FcnChain> state_terms;
    std::vector<FcnChain> input_terms;
    float predict_accel(const model_inputs_t &model_inputs, float V);
    float choose_V(const model_inputs_t &model_inputs, float accel);
};

namespace ModelFcns {
    // base functions
    class Vel : public ModelFcn {
        Vel() : ModelFcn(nullptr) {}; // singleton
    public:
        static ModelFcn *create() {static Vel instance; return &instance;}
        inline float call(const model_inputs_t &model_inputs) const override {
            return model_inputs.velocity;
        }
    };
    
    class One : public ModelFcn {
        One() : ModelFcn(nullptr) {}; // singleton
    public:
        static ModelFcn *create() {static One instance; return &instance;}
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
