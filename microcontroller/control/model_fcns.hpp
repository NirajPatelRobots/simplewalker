/* ModelFcns sub-classes implement specific functions. 
   They must be copy-constructable and assignable.
   TODO:
        get rid of mult?
        continuous threshold (static_fric), continuous sign (const fric)
        Punch
*/
#ifndef SIMPLEWALKER_PICO_MODEL_FCNS_HPP
#define SIMPLEWALKER_PICO_MODEL_FCNS_HPP
#include "motor_model.hpp"


namespace ModelFcns {
    // non-base functions
    class Sign : public ModelFcn {
        Sign() : ModelFcn() {};
    public:
        static ModelFcn *create() {return new Sign;}
        inline float call([[maybe_unused]] const model_inputs_t &_, float parentval) const override {
            return parentval == 0 ? 0.f : parentval > 0 ? 1.f : -1.f;
        }
    };
    
    class Mult : public ModelFcn {
        Mult() : ModelFcn(), mult(1) {};
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
        inline float call([[maybe_unused]] const model_inputs_t &_, float parent_val) const override {
            return mult * parent_val;
        }
    };
}  // namespace ModelFcns


// must stay in line with main computer motor_model_API_user.cpp
ModelFcn * (*const BASE_FUNCTIONS[])(void) = {
    ModelFcns::Vel::create,
    ModelFcns::One::create,
};


ModelFcn * (*const NONBASE_FUNCTIONS[])(void) = {
    ModelFcns::Sign::create,
    ModelFcns::Mult::create,
};
#endif  // SIMPLEWALKER_PICO_MODEL_FCNS_HPP
