// motor definitions and constants for simplewalker
// Created by Niraj, June 2023.

#ifndef SIMPLEWALKER_PICO_SIMPLEWALKER_MOTORS_HPP
#define SIMPLEWALKER_PICO_SIMPLEWALKER_MOTORS_HPP
#include "motor_IO.hpp"
#include "micro_parameters.h"
#include <memory>
#include <vector>

const int NUM_DCMOTORS = 4, NUM_SERVOMOTORS = 2, NUM_MOTORS = NUM_DCMOTORS + NUM_SERVOMOTORS;

enum Motornum {
    right_hip,
    right_knee,
    left_hip,
    left_knee,
    right_hip_out,
    left_hip_out,
};

bool is_servo(Motornum num) {
    return (num == right_hip_out || num == left_hip_out);
}

DCMotorPins right_hip_motor_pins{16, 17},
            right_knee_motor_pins{18, 19},
            left_hip_motor_pins{20, 21},
            left_knee_motor_pins{14, 15};
ServoMotorPins right_hip_out_servo_pin{12},
                left_hip_out_servo_pin{13};

const std::vector<motorIOSettings> SIMPLEWALKER_MOTOR_IO_SETTINGS {
        {"right_hip", motorType::DC, motorAngleSensorType::ADC,
         32, 915, //  raw_angle_min, raw_angle_max
         0.003, -484, // angle_scale, raw_angle_offset
         1, MAX_MOTOR_VOLTAGE,
         &right_hip_motor_pins},
        {"right_knee", motorType::DC, motorAngleSensorType::ADC,
         32, 915,
         0.003, -484,
         2, MAX_MOTOR_VOLTAGE,
         &right_knee_motor_pins}
};

#endif //SIMPLEWALKER_PICO_SIMPLEWALKER_MOTORS_HPP
