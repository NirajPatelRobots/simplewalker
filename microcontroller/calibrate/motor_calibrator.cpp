#include "motor_calibrator.hpp"
#include <stdio.h>
#include <math.h>
#include "pico/time.h"


MotorCalibrator::MotorCalibrator(const std::vector<motorIOSettings> _motor_settings,
    shared_ptr<MessageOutbox<MotorCalibrationStateMsg>> _state_outbox,
    int ADC_battery_voltage_channel, float ADC_battery_voltage_scale)
        : ADC(make_shared<ADCReader>()),
          motor_settings(_motor_settings),
          motors_IO(std::make_unique<MotorsIO>(motor_settings, ADC)),
          batteryVoltage(ADC->set_channel("batteryVoltage", ADC_battery_voltage_channel, 0, ADC_battery_voltage_scale)),
          state_outbox(_state_outbox),
          instructions(make_shared<MotorCalibrationTriggerMsg>()) {
        ADC->connect_SPI();
        motors_IO->initialize_ADC_channels();
        state_outbox->message.ID = MotorCalibrationStateMsgID;
        instructions->motorNum = 0;
        instructions->dt = 0.003;
        instructions->amplitude = 1.0;
        instructions->frequency = 0.0;
        instructions->text_output = 1;
    }

    int MotorCalibrator::calibrate_motor() {
        float angVel = 0.0;
        generator = make_signal_generator(MotorCalibrationInputType(instructions->input_signal_type),
                                          instructions->frequency * instructions->dt,
                                          instructions->amplitude);
        if (!generator) {
            printf("Invalid input type: %u", instructions->input_signal_type);
            return -3;
        }

        return_motor_to_start();
        status = MOTORCAL_RUNNING;
        int startup_stationary_samples = floor(0.75 / instructions->dt);
        int ending_stationary_samples = floor(0.75 / instructions->dt);
        absolute_time_t looptarget = get_absolute_time();
        absolute_time_t start_time = looptarget;
        for(int i = 0; i < startup_stationary_samples; i++) {
            if (!do_loop(0.0, angVel, looptarget, start_time))
                break;
        }
        while(!generator->is_finished) {
            if (!do_loop(generator->get_next_value(angVel), angVel, looptarget, start_time))
                break;
        }
        for(int i = 0; i < ending_stationary_samples; i++) {
            if (!do_loop(0.0, angVel, looptarget, start_time))
                break;
        }
        motors_IO->set_motor_voltage(instructions->motorNum, 0);
        sleep_us(floor(instructions->dt * 1e6));
        printf("finished calibration\n");
        status = MOTORCAL_IDLE;
        sleep_us(floor(instructions->dt * 1e6));
        return 0;
    }

    bool MotorCalibrator::do_loop(float V, float &angVel, absolute_time_t &looptarget, absolute_time_t start_time) {
        motors_IO->set_battery_voltage(ADC->read_ADC_scaled(batteryVoltage->channel_num));
        float angle = read_angle();
        if (!safely_set_motor(V, angle)) return false;
        angVel = calc_angvel(angle);
        report_result(angle, angVel, V, (looptarget - start_time) * 1e-6);
        looptarget = delayed_by_us(looptarget, (uint64_t)(instructions->dt * 1e6));
        sleep_until(looptarget);
        return true;
    }

    bool MotorCalibrator::safely_set_motor(float voltage, float angle) {
        motors_IO->set_motor_voltage(instructions->motorNum, voltage);
        if (fabs(angle) > instructions->max_displacement || angle < instructions->min_displacement) {
            printf("Test went out of range and was terminated\n");
            //return_motor_to_start();
            motors_IO->set_motor_voltage(instructions->motorNum, 0);
            return false;
        }
        return true;
    }

    float MotorCalibrator::read_angle() {
        return ADC->read_ADC_scaled(motor_settings[instructions->motorNum].sensor_channel_num);
    }

    float MotorCalibrator::calc_angvel(float angle) {
        static float lastAngle{0.0};
        float angVel = (angle - lastAngle) / instructions->dt;
        lastAngle = angle;
        return angVel;
    }

    void MotorCalibrator::return_motor_to_start() {
        float kp{3};
        int numInRange{0}, total_tries{0};
        status = MOTORCAL_CENTERING;
        if (instructions->text_output) printf("Returning motor to start...\n");
        while (numInRange < 10 && (float)(++total_tries) <= 2 / instructions->dt) {
            float angle = read_angle();
            calc_angvel(angle);
            float voltage = -kp * angle + (angle > 0 ? -1.f : 1.f);
            motors_IO->set_motor_voltage(instructions->motorNum, voltage);
            if (fabs(angle) < 0.02) {
                numInRange++;
            } else {
                numInRange = 0;
            }
//            report_result(angle, 0, voltage, (float)total_tries * instructions->dt);
            sleep_ms(floor(instructions->dt*1000));
        }
        motors_IO->set_motor_voltage(instructions->motorNum, 0);
        if (instructions->text_output) printf("Motor at start.\n");
    }

    void MotorCalibrator::report_result(float angle, float angvel, float voltage, float time) {
        if (instructions->text_output) {
            printf("%f,%f,%f,%f\n", time, voltage, angle, angvel);
        } else {
            state_outbox->message.angle = angle;
            state_outbox->message.status = status;
            state_outbox->message.timestamp_us = (uint32_t) (time * 1e6);
            state_outbox->message.voltage = voltage;
            state_outbox->send();
        }
    }
