/* Created by Niraj Oct 2021
 * TODO:
 *     DC in-in vs. direction-magnitude pin modes
 *     Variable pwm freq (quieter?)
 */

#ifndef SIMPLEWALKER_PICO_MOTOR_OUTPUT_HPP
#define SIMPLEWALKER_PICO_MOTOR_OUTPUT_HPP

class MotorOutput {
public:
    virtual int set_output(float fraction); // ensures abs(fraction) <= 1
    int err{};
    static const int ERR_DIFFERENT_PWM_SLICES;
};

class DCMotorOutput : public MotorOutput {
    const unsigned pin_forward, pin_reverse, pwmslice_num;
public:
    DCMotorOutput(unsigned _pin_forward, unsigned _pin_reverse);
    bool coast = false; // TODO default true?
    int set_output(float fraction) override; // ensures abs(fraction) <= 1
};

class ServoMotorOutput : public MotorOutput {
    const unsigned pin, pwmslice_num;
public:
    ServoMotorOutput(unsigned _pin);
    int set_output(float fraction) override; // ensures 0 <= fraction <= 1
};

float get_pwm_freq();

#endif