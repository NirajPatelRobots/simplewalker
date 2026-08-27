/* A motor model is a set of chains of modelFcns that predict the
 * change in state (acceleration) for any current state (velocity).
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
    ModelFcn *parent;
public:
    ModelFcn(ModelFcn *_parent) : parent(_parent) {}
    // set_param returns true if the param was set
    virtual bool set_param(unsigned int param_num, float value) {return false;};
    virtual float call(const model_inputs_t &model_inputs) const = 0;
};

struct FcnChain {
    ModelFcn *first_fcn;
    float weight;
    inline float call(const model_inputs_t &model_inputs) const {
        return weight * first_fcn->call(model_inputs);
    }
};

class MotorModel {
public:
    std::vector<FcnChain> functions;
    inline float predict(const model_inputs_t &model_inputs);
};


class VelocityModelFcn : public ModelFcn { // singleton
    VelocityModelFcn() : ModelFcn(nullptr) {};
public:
    static VelocityModelFcn& getInstance() {
        static VelocityModelFcn instance;
        return instance;
    }
    inline float call(const model_inputs_t &model_inputs) const override {
        return model_inputs.velocity;
    }
};
