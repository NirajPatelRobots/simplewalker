/* Motor Calibrator follows an input and reports voltage and angle measurements
Created October 2021, reworked late 2023, 2026
TODO:
    way to cancel calibration
    Standardize error responses?
    error if can't return to start
        detect wrong direction
    measure battery voltage variability?
    send reference signal from micro during calibration
    better names used consistently for input_signal_type and command_signal_type
    */
#ifndef SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
#define SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
#include "../control/motor_model.hpp"
#include "../motors/motor_IO.hpp"
#include "signal_generator.hpp"


class MotorCalibrator {
    float signal_to_voltage(float sig_value, float angVel);
    bool return_motor_to_start();
    bool safely_set_motor();
    int counter {0}, numInRange {0};
public:
    enum RUN_SECTION {SECTION_START, SECTION_GOING, SECTION_END} run_section;
    shared_ptr<ADCReader> ADC;
    const std::vector<motorIOSettings> motor_settings;
    std::unique_ptr<MotorsIO> motors_IO;
    shared_ptr<ADCChannel> batteryVoltage;
    MotorCalibrationStateMsg state;
    shared_ptr<MotorCalibrationTriggerMsg> instructions;
    std::unique_ptr<ExcitationSignalGenerator> generator;
    MotorModel motor_model;
    float angVel;
    MotorCalibrator(const std::vector<motorIOSettings> _motor_settings,
        int ADC_battery_voltage_channel, float ADC_battery_voltage_scale);
    int start_calibration();
    bool run_calibration_loop();
    void read_angle();
    float calc_angvel();
};

#endif  //SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
