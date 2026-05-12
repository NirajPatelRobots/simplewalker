/* calibrate the DC motor, reporting voltage and angle measurements.
Created October 2021, reworked late 2023
TODO:
    way to cancel calibration
    error if can't return to start
        detect wrong direction
    measure battery voltage variability?
    */


#include "../simplewalker_motors.hpp"
#include "../pico_comm.hpp"
#include "signal_generator.hpp"
#include <stdio.h>
#include <math.h>
#include "pico/time.h"


class MotorCalibrator {
public:
    std::unique_ptr<PicoCommunication> comm;
    shared_ptr<ADCReader> ADC;
    std::unique_ptr<MotorsIO> motors_IO;
    shared_ptr<ADCChannel> batteryVoltage;
    shared_ptr<MessageInbox<MotorCalibrationTriggerMsg>> trigger_inbox;
    shared_ptr<MessageOutbox<MotorCalibrationStateMsg>> state_outbox;
    shared_ptr<MotorCalibrationTriggerMsg> instructions;
    std::shared_ptr<ExcitationSignalGenerator> generator;
    int startup_stationary_samples = 50;
    MotorCalibrationStatus status {MOTORCAL_IDLE};

    MotorCalibrator()
        : comm(std::make_unique<PicoCommunication>()),
          ADC(make_shared<ADCReader>()),
          motors_IO(std::make_unique<MotorsIO>(SIMPLEWALKER_MOTOR_IO_SETTINGS, ADC)),
          batteryVoltage(ADC->set_channel("batteryVoltage", ADC_BATTERY_VOLTAGE_CHANNEL, 0, ADC_BATTERY_VOLTAGE_SCALE)),
          trigger_inbox(make_shared<MessageInbox<MotorCalibrationTriggerMsg>>(MotorCalibrationTriggerMsgID, *comm)),
          state_outbox(make_shared<MessageOutbox<MotorCalibrationStateMsg>>(MotorCalibrationStateMsgID, *comm)),
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

    int calibrate_motor() {
        float V = 0.0, lastV = 0.0, angle = 0.0, angVel = 0.0; //[V], [V], [rad], [rad/s]
        generator = make_signal_generator(MotorCalibrationInputType(instructions->input_signal_type),
                                          instructions->frequency * instructions->dt,
                                          instructions->amplitude);
        if (!generator) {
            printf("Invalid input type: %u", instructions->input_signal_type);
            return -3;
        }
        if (is_servo((Motornum)(instructions->motorNum))) {
            printf("implement servo pins\n");
            return -2;
        }

        return_motor_to_start();
        status = MOTORCAL_RUNNING;
        startup_stationary_samples = floor(1.0 / instructions->dt);
        absolute_time_t looptarget = get_absolute_time();
        absolute_time_t start_time = looptarget;
        for(int i = 0; i < startup_stationary_samples; i++) {
            angle = read_angle();
            angVel = calc_angvel(angle);
            report_result(angle, angVel, V, (looptarget - start_time) * 1e-6);
            looptarget = delayed_by_us(looptarget, (uint64_t)(instructions->dt * 1e6));
            sleep_until(looptarget);
        }
        while(!generator->is_finished) {
            V = generator->get_next_value(angVel);
            motors_IO->set_battery_voltage(ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL));
            if (!safely_set_motor(V, angle)) return -1;
            angle = read_angle();
            angVel = calc_angvel(angle);
            report_result(angle, angVel, lastV, (looptarget - start_time) * 1e-6);
            lastV = V; // shift i+1 because causality. So V[i] affects angle[i]
            looptarget = delayed_by_us(looptarget, (uint64_t)(instructions->dt * 1e6));
            sleep_until(looptarget);
        }
        sleep_us(floor(instructions->dt * 1e6));
        printf("finished calibration\n");
        status = MOTORCAL_IDLE;
        motors_IO->set_motor_voltage(instructions->motorNum, 0);
        sleep_us(floor(instructions->dt * 1e6));
        return 0;
    }

    bool safely_set_motor(float voltage, float angle) {
        motors_IO->set_motor_voltage(instructions->motorNum, voltage);
        if (fabs(angle) > instructions->max_displacement || angle < instructions->min_displacement) {
            printf("Test went out of range and was terminated\n");
            //return_motor_to_start();
            motors_IO->set_motor_voltage(instructions->motorNum, 0);
            return false;
        }
        return true;
    }

    float read_angle() {
        return ADC->read_ADC_scaled(SIMPLEWALKER_MOTOR_IO_SETTINGS[instructions->motorNum].sensor_channel_num);
    }

    float calc_angvel(float angle) {
        static float lastAngle{0.0};
        float angVel = (angle - lastAngle) / instructions->dt;
        lastAngle = angle;
        return angVel;
    }

    void return_motor_to_start() {
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

    void report_result(float angle, float angvel, float voltage, float time) {
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
};



int main() {
    MotorCalibrator calibrator{};
    while (1) {
        calibrator.comm->receive_messages();
        if (calibrator.trigger_inbox->get_newest(*calibrator.instructions) >= 0) {
            if (calibrator.instructions->motorNum >= SIMPLEWALKER_MOTOR_IO_SETTINGS.size()){
                printf("Motor number doesn't exist, using motor 0\n");
                calibrator.instructions->motorNum = 0;
            }
            printf("Calibrate motor %d; freq=%f, amp=%f, dt=%f\n",
                   calibrator.instructions->motorNum, calibrator.instructions->frequency,
                   calibrator.instructions->amplitude, calibrator.instructions->dt);
            sleep_ms(500);
            calibrator.calibrate_motor();
        } else {
            calibrator.instructions->dt = 0.5;
            float battery_voltage = calibrator.ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL);
            float angle = calibrator.read_angle();
            float angVel = calibrator.calc_angvel(angle);
            calibrator.motors_IO->set_motor_voltage(calibrator.instructions->motorNum, 0);
            calibrator.report_result(angle, angVel, battery_voltage, to_us_since_boot(get_absolute_time()) * 1e-6 + 1000);
            sleep_ms(500);
        }
    }
}
