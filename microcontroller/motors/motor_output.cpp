#include <math.h>
#include "motor_output.hpp"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

const unsigned PWM_WRAP = 125;
const float PWM_CLK_DIVIDER = 256;
const int MotorOutput::ERR_DIFFERENT_PWM_SLICES = 1;


float get_pwm_freq() {
    return 125e6f / PWM_CLK_DIVIDER / PWM_WRAP;
}


DCMotorOutput::DCMotorOutput(unsigned _pin_forward, unsigned _pin_reverse)
    : pin_forward(_pin_forward), pin_reverse(_pin_reverse), pwmslice_num(pwm_gpio_to_slice_num(pin_forward)) {
    err = (pwm_gpio_to_slice_num(pin_reverse) == pwmslice_num) ? 0 : ERR_DIFFERENT_PWM_SLICES;
    gpio_set_function(pin_forward, GPIO_FUNC_PWM);
    gpio_set_function(pin_reverse, GPIO_FUNC_PWM);
    pwm_set_wrap(pwmslice_num, PWM_WRAP);
    pwm_set_enabled(pwmslice_num, true);
    pwm_set_clkdiv(pwmslice_num, PWM_CLK_DIVIDER);
}

int DCMotorOutput::set_output(float fraction) {
    if (fraction > 1.) {
        fraction = 1.;
    } else if (fraction < -1.) {
        fraction = -1.;
    }
    int pwm_command = (int)round(fraction * PWM_WRAP); // TODO is this cast avoiding pwm_command = 0?
    if (pwm_command > 0) {
        if (coast)
            pwm_set_both_levels(pwmslice_num, pwm_command, 0);
        else
            pwm_set_both_levels(pwmslice_num, PWM_WRAP, PWM_WRAP-pwm_command);
    } else {
        if (coast)
            pwm_set_both_levels(pwmslice_num, 0, -pwm_command);
        else
            pwm_set_both_levels(pwmslice_num, PWM_WRAP+pwm_command, PWM_WRAP);
    }
    return pwm_command;
}


ServoMotorOutput::ServoMotorOutput(unsigned _pin)
    : pin(_pin), pwmslice_num(pwm_gpio_to_slice_num(_pin)) {
    gpio_set_function(pin, GPIO_FUNC_PWM);
    pwm_set_wrap(pwmslice_num, PWM_WRAP);
    pwm_set_enabled(pwmslice_num, true);
    pwm_set_clkdiv(pwmslice_num, PWM_CLK_DIVIDER);
    pwm_set_gpio_level(pin, PWM_WRAP / 2);
}

int ServoMotorOutput::set_output(float fraction) {
    int pwm_command = (int)roundf(fraction * PWM_WRAP);
    if (pwm_command > PWM_WRAP) pwm_command = PWM_WRAP;
    if (pwm_command < 0) pwm_command = 0;
    pwm_set_gpio_level(pin, pwm_command);
    return pwm_command;
}
