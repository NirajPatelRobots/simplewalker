/* Input/Output for motors
 * Created by Niraj, Jan 2024.
 *
 * How to set up the object
 * 1. create sensor objects (ADC)
 * 2. Create a vector of MotorIOSettings with the motors you want (simplewalker_motors.hpp)
 * 3. create a battery_voltage ADC channel
 *
 * how to use in loop:
 * 1. read the sensors
 * 2. call set_battery_voltage with the updated battery voltage
 * 3. set the motor voltages with set_motor_voltage
 *    if it returns false, it's out of range
 *
 * TODO:
 *     set servo angle
 *     angle accessor
 *     error code and error LED
 *     DC in-in vs. phase-enable motorTypes
 *     BLDC, eventually
 */

#ifndef SIMPLEWALKER_PICO_MOTOR_IO_HPP
#define SIMPLEWALKER_PICO_MOTOR_IO_HPP
#include "motor_output.hpp"
#include "ADC_reader.hpp"
#include <string>
#include <vector>


enum class motorType {DC, Servo};
enum class motorAngleSensorType {None = 0, ADC, Encoder};
enum class motorCurrentSensorType {None = 0, RELATIVE};

struct motorIOSettings {
    std::string name;
    motorType type;
    motorAngleSensorType angleSensorType;
    int raw_angle_min, raw_angle_max;
    float angle_scale;  //  angle = angle_scale * (sensor_reading + raw_angle_offset)
    int raw_angle_offset;
    int sensor_channel_num;
    float max_voltage;
    void * motor_pin_settings;
};

struct DCMotorPins {
    uint pin_forward, pin_reverse;
};
struct ServoMotorPins {
    uint pin;
};

class MotorsIO {
    const std::vector<motorIOSettings> all_motor_settings_;
    shared_ptr<ADCReader> ADC_;
    std::vector<std::unique_ptr<MotorOutput>> motor_outputs;
    float battery_voltage_;
    int error{};
    uint error_LED_pin{};
    bool set_motor_error(int motor_num, float fraction, int error_num);
public:
    MotorsIO(const std::vector<motorIOSettings> &_all_motor_settings,
             shared_ptr<ADCReader> &ADC);
    bool set_motor_voltage(int motor_num, float voltage); // returns false if it couldn't be set

    void set_battery_voltage(float voltage) {if (voltage > 0) battery_voltage_ = voltage;}

    void initialize_ADC_channels(); // ADC_->set_channel for motors with ADC
    int error_code() const {return error;}
};


#endif // SIMPLEWALKER_PICO_MOTOR_IO_HPP
