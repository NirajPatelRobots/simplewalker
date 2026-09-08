/* Motor Calibrator follows an input and reports voltage and angle measurements
Created October 2021, reworked late 2023
TODO:
    way to cancel calibration
    Invalid scale values error?
    error if can't return to start
        detect wrong direction
    measure battery voltage variability?
    different input mode for acceleration, uses motor model
    */
#ifndef SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
#define SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
#include "../communication/pico_comm.hpp"
#include "../motors/motor_IO.hpp"
#include "signal_generator.hpp"


class MotorCalibrator {
public:
    shared_ptr<ADCReader> ADC;
    const std::vector<motorIOSettings> motor_settings;
    std::unique_ptr<MotorsIO> motors_IO;
    shared_ptr<ADCChannel> batteryVoltage;
    shared_ptr<MessageOutbox<MotorCalibrationStateMsg>> state_outbox;
    shared_ptr<MotorCalibrationTriggerMsg> instructions;
    std::unique_ptr<ExcitationSignalGenerator> generator;
    MotorCalibrationStatus status {MOTORCAL_IDLE};
    MotorCalibrator(const std::vector<motorIOSettings> _motor_settings,
        shared_ptr<MessageOutbox<MotorCalibrationStateMsg>> _state_outbox,
        int ADC_battery_voltage_channel, float ADC_battery_voltage_scale);
    int calibrate_motor();
    bool do_loop(float V, float &angVel, absolute_time_t &looptarget, absolute_time_t start_time);
    bool safely_set_motor(float voltage, float angle);
    float read_angle();
    float calc_angvel(float angle);
    void return_motor_to_start();
    void report_result(float angle, float angvel, float voltage, float time);
};

#endif  //SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
