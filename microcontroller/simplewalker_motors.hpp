// motor definitions and constants for simplewalker
// Created by Niraj, June 2023.

#ifndef SIMPLEWALKER_PICO_SIMPLEWALKER_MOTORS_HPP
#define SIMPLEWALKER_PICO_SIMPLEWALKER_MOTORS_HPP
#include "motor_output.hpp"
#include <memory>
#include <vector>

const int NUM_DCMOTORS = 4, NUM_SERVOMOTORS = 2, NUM_MOTORS = NUM_DCMOTORS + NUM_SERVOMOTORS;

uint pin_forward[NUM_DCMOTORS] = {16, 18, 20, 14};
uint pin_reverse[NUM_DCMOTORS] = {17, 19, 21, 15};
uint pin_servo[NUM_SERVOMOTORS] = {12, 13};
uint servo_offset[NUM_SERVOMOTORS] = {0, 0};
enum Motornum {
    right_hip,
    right_knee,
    left_hip,
    left_knee,
    right_hip_out,
    left_hip_out,
};

class Simplewalker_MotorOutput {
public:
    std::vector<std::unique_ptr<MotorOutput>> motors;
    Simplewalker_MotorOutput()
    {
        for (int i = 0; i < NUM_DCMOTORS; ++i) {
            motors.push_back(std::make_unique<DCMotorOutput>(pin_forward[i], pin_reverse[i]));
        }
        for (int i = 0; i < NUM_SERVOMOTORS; ++i) {
            motors.push_back(std::make_unique<ServoMotorOutput>(pin_servo[i], servo_offset[i]));
        }
    }
    void set(Motornum num, float fraction) {
        motors[num]->set_output(fraction);
    }
};

#endif //SIMPLEWALKER_PICO_SIMPLEWALKER_MOTORS_HPP
