/* Created by Niraj Oct 2021 */

#ifndef SIMPLEWALKER_PICO_MOTOR_OUTPUT_HPP
#define SIMPLEWALKER_PICO_MOTOR_OUTPUT_HPP
#include "pico/stdlib.h"

class MotorOutput {
public:
    virtual int set_output(float fraction); // ensures abs(fraction) <= 1
    int err{};
    static const int ERR_DIFFERENT_PWM_SLICES;
};

class DCMotorOutput : public MotorOutput {
    const uint pin_forward, pin_reverse, pwmslice_num;
public:
    DCMotorOutput(uint _pin_forward, uint _pin_reverse);
    int set_output(float fraction) override; // ensures abs(fraction) <= 1
};

class ServoMotorOutput : public MotorOutput {
    const uint pin, pwmslice_num;
public:
    ServoMotorOutput(uint _pin);
    int set_output(float fraction) override; // ensures 0 <= fraction <= 1
};

#endif