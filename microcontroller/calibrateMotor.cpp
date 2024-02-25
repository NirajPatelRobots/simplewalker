/* calibrate the DC motor, reporting voltage and angle measurements.
Created October 2021, reworked late 2023
TODO:
    leg calibration
    take time measurements for better timing
    remove voltage causality time shift, do that in analysis?
    error if can't return to start
    reset excitationVoltage if can't return to start
    */


#include "simplewalker_motors.hpp"
#include "pico_comm.hpp"
#include "../communication/messages.h"
#include <stdio.h>
#include <math.h>


class ExcitationVoltageGenerator {
public:
    int place, loop_num, extra_count;
void reset_state() {
    place = 0;
    loop_num = 0;
    extra_count = 0;
}
/* Choose voltage for test input to the motor.
output is float *V, returns 0 if waveform continues, 1 if it's finished
Starts at counter = 0, creates a waveform over each time it's called
Made up of a sinusoid then a square wave */
int excitationVoltage(float frequency_scale, float amplitude_scale, float *V) {
    float freq = 200. * frequency_scale, amp = amplitude_scale;
    float wiggle_amp = 0.0 * amp, wiggle_freq = 0.7 * freq / amp;
    int num_loops = 6, num_square = 3, square_length = (int)(0.25 / frequency_scale);

    if ( loop_num < num_loops) { // loops
        float loop_amp = (float)(loop_num+1)/(num_loops+1) * amp;
        float loop_freq = freq / loop_amp;
        if (loop_num % 2 == 1) loop_amp *= -1;
        *V = loop_amp * sin(loop_freq * 0.01 * place);
        if (place++ >= 314.0 / loop_freq) {
            place = 0;
            loop_num++;
        }
    } else if (loop_num < 2 * num_loops) { // loops with wiggles
        float loop_amp = (1.0-(float)(loop_num-num_loops+1)/(num_loops+1)) * amp;
        float loop_freq = freq / loop_amp;
        if (loop_num % 2 == 1) loop_amp *= -1;
        *V = loop_amp * sin(loop_freq * 0.01 * place) + wiggle_amp * sin(wiggle_freq * extra_count++);
        if (place++ >= 314.0 / loop_freq) {
            place = 0;
            loop_num++;
        }
    } else {
        int square_num = loop_num - (2 * num_loops);
        ++place;
        if (square_num < num_square) {
            if (place < square_length)           *V = amp;
            else if (place < 3*square_length)    *V = -amp;
            else                                 *V = amp;
            if (place >= 4*square_length) {
                loop_num++;
                place = 0;
            }
        } else {
            if (place < square_length)      *V = 0.0;
            else { //finally done
                reset_state();
                return 1;
            }
        }
    }
    return 0;
}
};


class MotorCalibrator {
public:
    std::unique_ptr<PicoCommunication> comm;
    shared_ptr<ADCReader> ADC;
    std::unique_ptr<MotorsIO> motors_IO;
    shared_ptr<ADCChannel> batteryVoltage;
    shared_ptr<MessageInbox<MotorCalibrationTriggerMsg>> trigger_inbox;
    shared_ptr<MessageOutbox<MotorCalibrationStateMsg>> state_outbox;
    shared_ptr<MotorCalibrationTriggerMsg> instructions;
    ExcitationVoltageGenerator excitationVoltageGenerator;
    const float const_voltage_duration{2};
    int send_skip_iteration_counter;

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
        float V, lastV = 0.0, angle = 0.0, angVel; //[V], [V], [rad], [rad/s]
        int i = 0;
        send_skip_iteration_counter = 0;
        excitationVoltageGenerator.reset_state();

        if (is_servo((Motornum)(instructions->motorNum))) {
            printf("implement servo pins\n");
            return -2;
        }

        return_motor_to_start();
        absolute_time_t looptarget = get_absolute_time();
        bool input_finished;
        do {
            if (instructions->frequency <= 0.001) {
                input_finished = (i * instructions->dt > const_voltage_duration);
                V = instructions->amplitude;
            }
            else
                input_finished = excitationVoltageGenerator.excitationVoltage(
                        instructions->frequency * instructions->dt, instructions->amplitude, &V);
            motors_IO->set_battery_voltage(ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL));
            if (!safely_set_motor(V, angle)) return -1;
            angle = read_angle();
            angVel = calc_angvel(angle);
            report_result(angle, angVel, lastV, instructions->dt * i);
            i++;
            lastV = V; // shift i+1 because causality. So V[i] affects angle[i]
            looptarget = delayed_by_us(looptarget, (uint64_t)(instructions->dt * 1e6));
            sleep_until(looptarget);
        } while(!input_finished);
        printf("finished calibration\n");
        motors_IO->set_motor_voltage(instructions->motorNum, 0);
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
        if (instructions->text_output) printf("Returning motor to start...\n");
        while (numInRange < 10 && ++total_tries <= 10 / instructions->dt) {
            float angle = read_angle() + 0.25;
            float voltage = -kp * angle + (angle > 0 ? -1.5 : 1.5);
            motors_IO->set_motor_voltage(instructions->motorNum, voltage);
            if (fabs(angle) < 0.1) {
                numInRange++;
            } else {
                printf("%f %f; ", angle, voltage);
                numInRange = 0;
            }
            sleep_ms(instructions->dt*1000);
        }
        motors_IO->set_motor_voltage(instructions->motorNum, 0);
    }

    void report_result(float angle, float angvel, float voltage, float time) {
        if (send_skip_iteration_counter++ >= instructions->send_skip_iterations) {
            if (instructions->text_output) {
                printf("%f,%f,%f,%f\n", time, voltage, angle, angvel);
            } else {
                state_outbox->message.angle = angle;
                state_outbox->message.angvel = angvel;
                state_outbox->message.timestamp_us = (uint32_t) (time * 1e6);
                state_outbox->message.voltage = voltage;
                state_outbox->send();
            }
            send_skip_iteration_counter = 0;
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
            float angle = calibrator.read_angle();
            float angVel = calibrator.calc_angvel(angle);
            calibrator.motors_IO->set_motor_voltage(calibrator.instructions->motorNum, 0);
            calibrator.report_result(angle, angVel, 0, to_us_since_boot(get_absolute_time()) * 1e-6);
            sleep_ms(500);
        }
    }
}
