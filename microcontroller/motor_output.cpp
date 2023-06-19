#include "motor_output.hpp"
#include "hardware/pwm.h"

# define M_PI_2 1.5707963267948966
const float M_PI = 3.141592653589793;

const uint PWM_WRAP = 100;
const int MotorOutput::ERR_DIFFERENT_PWM_SLICES = 1;


DCMotorOutput::DCMotorOutput(uint _pin_forward, uint _pin_reverse)
    : pin_forward(_pin_forward), pin_reverse(_pin_reverse), pwmslice_num(pwm_gpio_to_slice_num(pin_forward)) {
    err = (pwm_gpio_to_slice_num(pin_reverse) == pwmslice_num) ? 0 : ERR_DIFFERENT_PWM_SLICES;
    gpio_set_function(pin_forward, GPIO_FUNC_PWM);
    gpio_set_function(pin_reverse, GPIO_FUNC_PWM);
    pwm_set_wrap(pwmslice_num, PWM_WRAP);
    pwm_set_enabled(pwmslice_num, true);
    pwm_set_clkdiv_int_frac(pwmslice_num, 16, 0); //give the processor a break
}

int DCMotorOutput::set_output(float fraction) {
    if (fraction > 1.) {
        fraction = 1.;
    } else if (fraction < -1.) {
        fraction = -1.;
    }
    int pwm_command = (int)(fraction * PWM_WRAP);
    if (pwm_command > 0) {
        pwm_set_both_levels(pwmslice_num, PWM_WRAP, PWM_WRAP-pwm_command);
    } else {
        pwm_set_both_levels(pwmslice_num, PWM_WRAP+pwm_command, PWM_WRAP);
    }
    return pwm_command;
}


ServoMotorOutput::ServoMotorOutput(uint _pin)
    : pin(_pin), pwmslice_num(pwm_gpio_to_slice_num(_pin)) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_wrap(pwmslice_num, PWM_WRAP);
    pwm_set_enabled(pwmslice_num, true);
    pwm_set_clkdiv_int_frac(pwmslice_num, 16, 0); //give the processor a break
    pwm_set_gpio_level(pin, PWM_WRAP / 2);
}

int ServoMotorOutput::set_output(float fraction) {
    int pwm_command = (int)(fraction * PWM_WRAP / M_PI);
    if (pwm_command > PWM_WRAP) pwm_command = PWM_WRAP;
    if (pwm_command < 0) pwm_command = 0;
    pwm_set_gpio_level(pin, pwm_command);
    return pwm_command;
}
