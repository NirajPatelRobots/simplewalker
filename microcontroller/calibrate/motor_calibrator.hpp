/* Motor Calibrator follows an input and reports voltage and angle measurements
Created October 2021, reworked late 2023, 2026
TODO:
    Standardize error responses?
    error if can't return to start
        detect wrong direction
    measure battery voltage variability?
    configure which signal to send? (reference, velocity, battery voltage)
    */
#ifndef SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
#define SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
#include "../control/motor_model.hpp"
#include "../filter/micro_filter.hpp"
#include "../motors/motor_IO.hpp"
#include "signal_generator.hpp"


const int VEL_FILT_DESPIKE_N = 4, VEL_FILT_AVG_N = 10;
const bool BYPASS_FILT_DESPIKE = false, BYPASS_FILT_AVG = false;


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
    MotorCalibrationStateMsg state;
    shared_ptr<MotorCalibrationTriggerMsg> instructions;
    std::unique_ptr<ExcitationSignalGenerator> generator;
    MotorModel motor_model;
    SymmDespiker<VEL_FILT_DESPIKE_N> despiker;
    MovingAverage<VEL_FILT_AVG_N> movingAvg;
    float angVel;
    MotorCalibrator(const std::vector<motorIOSettings> _motor_settings);
    int start_calibration();
    bool run_calibration_loop();
    void read_angle();
    void calc_angvel();
};

#endif  //SIMPLEWALKER_PICO_MOTOR_CALIBRATOR_HPP
