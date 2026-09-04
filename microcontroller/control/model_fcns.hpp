#include "motor_model.hpp"


namespace ModelFcns {
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
    
    class Mult : public ModelFcn {
        Mult() : ModelFcn(nullptr), mult(1) {};
        float mult;
    public:
        static ModelFcn *create() {return new Mult;}
        bool set_param(unsigned num, float value) override {
            if (num == 0) {
                mult = value;
                return true;
            }
            return false;
        }
        inline float call(const model_inputs_t &model_inputs) const override {
            return mult * parent->call(model_inputs);
        }
    };
}  // namespace ModelFcns
