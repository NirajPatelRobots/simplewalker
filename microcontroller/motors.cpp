#include "motors.hpp"
# define M_PI_2 1.5707963267948966

/* TODO:
    hip_out offset and scaling */


uint Motors::pin_forward[NUM_MOTORS] = {16, 18, 20, 14, 12, 13};
uint Motors::pin_reverse[NUM_MOTORS] = {17, 19, 21, 15, 12, 13};

Motors::Motors (void) {
    pwmwrap = 100;
    for (int i = 0; i < NUM_MOTORS; i++) {
        gpio_set_function(pin_forward[i], GPIO_FUNC_PWM);
        gpio_set_function(pin_reverse[i], GPIO_FUNC_PWM);
        pwmslice_num[i] = pwm_gpio_to_slice_num(pin_forward[i]);
        pwm_set_wrap(pwmslice_num[i], pwmwrap);
        pwm_set_enabled(pwmslice_num[i], true);
        pwm_set_clkdiv_int_frac(pwmslice_num[i], 16, 0); //give the processor a break
    }
    angle0offsetR = pwmwrap * 0.0;
    angle0offsetL = pwmwrap * 0.0;
    pwm_set_gpio_level(pin_forward[right_hip_out], angle0offsetR);
    pwm_set_gpio_level(pin_forward[left_hip_out], angle0offsetL);
}

void Motors::setMotor(Motornum num, float command) {
    /* set the duty cycle of the motor. num is motor number.
    for hip and knee motors, command is duty cycle with magnitude <= 1.
    for hip_out, command is angle [0, pi/2] */
    if (command > 1.) {
        command = 1.;
    } else if (command < -1.) {
        command = -1.;
    }
    int pwm_command = (int)(command * pwmwrap);
    if (num == right_hip_out) {
        pwm_set_gpio_level(pin_forward[right_hip_out], pwm_command + angle0offsetR);
    } else if (num == left_hip_out) {
        pwm_set_gpio_level(pin_forward[left_hip_out], pwm_command + angle0offsetL);
    } else {
        if (pwm_command > 0) {
            pwm_set_both_levels(pwmslice_num[num], pwmwrap, pwmwrap-pwm_command);
        } else {
            pwm_set_both_levels(pwmslice_num[num], pwmwrap+pwm_command, pwmwrap);
        }
    }
}

