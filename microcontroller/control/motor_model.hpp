/* See README.md #"Motor Models"
 * TODO:
 *      FcnChain enforce initial base_fcn is a candidate base function (doesn't need parent)
 *      ModelFcns different data structure instead of subclass?
 *          Template<size_t num_params, float fcn(const model_inputs_t &model_inputs)>?
 *          std::pair<size_t, float fcn(const model_inputs_t &model_inputs)>?
 *      Programatically create function chains from a command API
 *      State API and Input API. (TODO: Combine?) (TODO: Also read model API?)
        - `Create_Term(base_fcn_id, weight)` -> `term_number`
        - `Add_Function(term_number, nonbase_fcn_id)`
        - `Set_Parameter(term_number)` TODO: how set if not first?
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
    virtual float call(const model_inputs_t &model_inputs) const = 0;
    ModelFcn *get_parent() const {return parent;}
};

// FcnChain owns the memory for its first_fcn, functions own their parent memory
struct FcnChain {
    FcnChain(ModelFcn *base_fcn, float _weight) : first_fcn(base_fcn), weight(_weight) {}
    ~FcnChain() {delete first_fcn;}
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
