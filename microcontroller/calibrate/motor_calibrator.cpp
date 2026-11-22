#include "motor_calibrator.hpp"
#include <stdio.h>
#include <math.h>


const char *const wave_shape_names[] = {"sine", "square", "deadband", "constant"}; 
const char *const signal_type_names[] = {"voltage", "accel", "position"}; 


MotorCalibrator::MotorCalibrator(const std::vector<motorIOSettings> _motor_settings)
        : ADC(make_shared<ADCReader>()),
          motor_settings(_motor_settings),
          motors_IO(std::make_unique<MotorsIO>(motor_settings, ADC)),
          state(),
          instructions(make_shared<MotorCalibrationTriggerMsg>()),
          motor_model(),
          angVel() {
        ADC->connect_SPI();
        motors_IO->initialize_ADC_channels();
        state.ID = MotorCalibrationStateMsgID;
        instructions->motorNum = 0;
        instructions->dt = 0.003;
        instructions->amplitude = 1.0;
        instructions->frequency = 0.0;
        instructions->text_output = 1;
    }

    int MotorCalibrator::start_calibration() {
        if (instructions->command_signal_type > CAL_SIGNAL_POS) {
            printf("Error: Invalid signal type\n");
            return -1;
        }
        if (instructions->wave_shape == CAL_INPUT_DEADBAND 
         && instructions->command_signal_type != CAL_SIGNAL_VOLTAGE) {
            printf("Error: Deadband is only possible for Voltage signals.\n");
            return -2;
        }
        if (state.status != MOTORCAL_IDLE) {
            printf("Error: started calibration before another one completed\n");
            state.status = MOTORCAL_IDLE;
            return -4;
        }
        generator = make_signal_generator(MotorCalibrationWaveShape(instructions->wave_shape),
                                          instructions->frequency * instructions->dt,
                                          instructions->amplitude);
        if (!generator) {
            printf("Invalid input type: %u", instructions->wave_shape);
            return -3;
        }
        printf("Signal %s tracking waveform %s\n", signal_type_names[instructions->command_signal_type],
                                                    wave_shape_names[instructions->wave_shape]);
        state.status = MOTORCAL_CENTERING;
        run_section = SECTION_START;
        counter = 0; numInRange = 0;
        if (instructions->text_output) printf("Returning motor to start...\n");
        return 0;
    }

    bool MotorCalibrator::run_calibration_loop() {
        if (state.status == MOTORCAL_IDLE) {
            printf("Error: ran calibration before starting\n");
            return false;
        }
        read_angle();
        calc_angvel();
        motors_IO->set_battery_voltage(ADC->read_ADC_scaled("batteryVoltage"));
        if (state.status == MOTORCAL_CENTERING) {
            return return_motor_to_start();
        }
        int startup_stationary_samples = floor(0.75 / instructions->dt);
        int ending_stationary_samples = floor(0.75 / instructions->dt);
        if (run_section == SECTION_START) {
            state.reference = 0;
            state.voltage = 0;
            if (counter++ == startup_stationary_samples) {
                run_section = SECTION_GOING;
            }
        } else if (run_section == SECTION_GOING) {
            state.reference = generator->get_next_value(angVel);
            state.voltage = signal_to_voltage(state.reference, angVel);
            if(generator->is_finished) {
                run_section = SECTION_END;
                counter = 0;
            }
        } else {
            state.reference = 0;
            state.voltage = 0;
            if (counter++ == ending_stationary_samples) {
                printf("finished calibration\n");
                state.status = MOTORCAL_IDLE;
            }
        }
        return safely_set_motor();
    }

    float MotorCalibrator::signal_to_voltage(float sig_value, float angVel) {
        float voltage;
        switch (instructions->command_signal_type) {
        case CAL_SIGNAL_VOLTAGE:
            voltage = sig_value;
            break;
        case CAL_SIGNAL_ACCEL:
            voltage = motor_model.choose_V({.velocity = angVel}, sig_value);
            break;
        default:
            printf("ERROR: command signal type does not exist\n");
            voltage = 0;
        }
        return voltage;
    }

    bool MotorCalibrator::safely_set_motor() {
        if (state.angle > instructions->max_displacement || state.angle < instructions->min_displacement) {
            printf("Test went out of range and was terminated\n");
            state.voltage = 0;
            motors_IO->set_motor_voltage(instructions->motorNum, 0);
            return false;
        }
        return motors_IO->set_motor_voltage(instructions->motorNum, state.voltage);
    }

    void MotorCalibrator::read_angle() {
        state.angle = ADC->read_ADC_scaled(motor_settings[instructions->motorNum].sensor_channel_num);
    }

    void MotorCalibrator::calc_angvel() {
        static float lastAngle{0.0};
        float despiked, raw_angvel = (state.angle - lastAngle) / instructions->dt;
        if constexpr (BYPASS_FILT_DESPIKE) {
            despiked = raw_angvel;
        } else {
            despiker.new_value(raw_angvel);
            despiked = despiker.get();
        }
        if constexpr (BYPASS_FILT_AVG) {
            angVel = despiked;
        } else {
            movingAvg.new_value(despiked);
            angVel = movingAvg.get();
        }
        lastAngle = state.angle;
    }

    bool MotorCalibrator::return_motor_to_start() {
        float kp{3};
        float angle_tol = 0.02;
        state.reference = 0;
        if (numInRange < 10 && (float)(++counter) <= 2.f / instructions->dt) {
            if (fabs(state.angle) < angle_tol) {
                state.voltage = 0;
                numInRange++;
            } else {
                state.voltage = (state.angle > angle_tol) ? -1.f : 1.f;
                numInRange = 0;
            }
            return motors_IO->set_motor_voltage(instructions->motorNum, state.voltage);
        } else {
            if (instructions->text_output) printf("Motor at start.\n");
            counter = 0;
            state.status = MOTORCAL_RUNNING;
            state.voltage = 0;
            return motors_IO->set_motor_voltage(instructions->motorNum, 0);
        }
    }
