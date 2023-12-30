/* calibrate the DC motor, reporting voltage and angle measurements.
Created October 2021, reworked late 2023
TODO:
    leg calibration
    take time measurements for better timing
    remove voltage causality time shift, do that in analysis?
    */


#include "motor_output.hpp"
#include "ADC_reader.hpp"
#include "pico_comm.hpp"
#include "micro_parameters.h"
#include "../communication/messages.h"
#include <stdio.h>
#include <math.h>

const float const_voltage_duration{2};


/* Choose voltage for test input to the motor.
output is float *V, returns 0 if waveform continues, 1 if it's finished
Starts at counter = 0, creates a waveform over each time it's called
Made up of a sinusoid then a square wave */
int excitationVoltage(float frequency_scale, float amplitude_scale, float *V) {
    float freq = 500. * frequency_scale, amp = 2. * amplitude_scale;
    int num_loops = 6, num_square = 3, square_length = (int)(200.0 / freq);
    static int place, loop_num, extra_count;

    if ( loop_num < num_loops) { // loops
        float loop_amp = (loop_num+1)/(num_loops+1) * amp;
        float loop_freq = freq / loop_amp;
        if (loop_num % 2 == 1) loop_amp *= -1;
        *V = loop_amp * sin(loop_freq * 0.01 * place);
        if (place < 314.0 / loop_freq) {
            place++;
        } else {
            place = 0;
            loop_num++;
        }
    } else if (loop_num < 2 * num_loops) { // loops with wiggles
        float loop_amp = (1-(loop_num-num_loops+1)/(num_loops+1)) * amp;
        float loop_freq = freq / loop_amp;
        if (loop_num % 2 == 1) loop_amp *= -1;
        *V = loop_amp * sin(loop_freq * 0.01 * place) + 0.1 * amp * sin(13 * freq / amp * extra_count++);
        if (place < 314.0 / loop_freq) {
            place++;
        } else {
            place = 0;
            loop_num++;
        }
    } else {
        int square_num = (loop_num - (2 * num_loops)) / (4 *square_length);
        if (square_num < num_square) {
            if (place < square_length)           *V = amp;
            else if (place < 3*square_length)    *V = -amp;
            else                                 *V = amp;
            if (++place == 4*square_length) {
                loop_num++;
                place = 0;
            }
        } else {
            if (place++ < square_length)      *V = 0.0;
            else { //finally done
                loop_num = 0;
                place = 0;
                extra_count = 0;
                return 1;
            }
        }
    }
    return 0;
}


class MotorCalibrator {
public:
    std::unique_ptr<PicoCommunication> comm;
    shared_ptr<MotorOutput> motor;
    ADCReader ADC;
    shared_ptr<ADCChannel> batteryVoltage;
    shared_ptr<MessageInbox<MotorCalibrationTriggerMsg>> trigger_inbox;
    shared_ptr<MessageOutbox<MotorCalibrationStateMsg>> state_outbox;
    shared_ptr<MotorCalibrationTriggerMsg> instructions;
    float maxDisplacement{0.45 * M_PI}; // [rad]
    int send_skip_iteration_counter;

    MotorCalibrator()
        : comm(std::make_unique<PicoCommunication>()),
          batteryVoltage(ADC.set_channel("batteryVoltage", 0, 0, ADC_BATTERY_VOLTAGE_SCALE)),
          //trigger_inbox(make_shared<MessageInbox<MotorCalibrationTriggerMsg>>(MotorCalibrationTriggerMsgID, *comm)),
          state_outbox(make_shared<MessageOutbox<MotorCalibrationStateMsg>>(MotorCalibrationStateMsgID, *comm)),
          instructions(make_shared<MotorCalibrationTriggerMsg>()) {
        ADC.connect_SPI();
        //comm->add_inbox(trigger_inbox.get());
        state_outbox->message.ID = MotorCalibrationStateMsgID;
        instructions->motorNum = 0;
        instructions->dt = 0.003;
        instructions->amplitude = 1.0;
        instructions->frequency = 0.0;
        instructions->text_output = 1;
    }

    int calibrate_motor() {
        float V, lastV = 0.0, angle, angVel = 0.0; //[V], [V], [rad], [rad/s]
        int i = 0;
        send_skip_iteration_counter = 0;

        return_motor_to_start();
        absolute_time_t looptarget = get_absolute_time();
        ADC.set_channel("motor", instructions->motorNum + 1, 0, 1);
        bool input_finished;
        do {
            if (instructions->frequency <= 0.001) {
                input_finished = (i > const_voltage_duration * instructions->dt);
                V = instructions->amplitude;
            }
            else
                input_finished = excitationVoltage(instructions->frequency * instructions->dt, instructions->amplitude, &V);
            float V_bat = 3.3;
            if (V > V_bat) { // constrain to battery voltage
                V = V_bat;
            } else if (V < -V_bat) {
                V = -V_bat;
            }
            if (!safely_set_motor(V / V_bat, angle)) return -1;
            angle = read_angle();
            angVel = calc_angvel(angle);
            report_result(angle, angVel, lastV, instructions->dt * i);
            i++;
            lastV = V; // shift i+1 because causality. So V[i] affects angle[i]
            looptarget = delayed_by_us(looptarget, ADMIN_DT_US);
            sleep_until(looptarget);
        } while(!input_finished);
        return 0;
    }

    bool safely_set_motor(float fraction, float angle) {
        motor->set_output(fraction);
        if (fabs(angle) > instructions->max_displacement || fabs(angle) < instructions->min_displacement) {
            printf("Test went out of range and was terminated\n");
            return_motor_to_start();
            return false;
        }
        return true;
    }

    float read_angle() {
        return ADC.read_ADC_scaled(instructions->motorNum + 1);
    }

    float calc_angvel(float angle) {
        static float lastAngle{0.0};
        float angVel = (angle - lastAngle) / instructions->dt;
        lastAngle = angle;
        return angVel;
    }

    void return_motor_to_start() {
        float kp = 0.002;
        int numInRange = 0;
        while (numInRange < 10) {
            float angle = ADC.read_ADC_scaled(instructions->motorNum+1);
            motor->set_output(-kp * angle);
            if (fabs(angle) < 0.01) {
                numInRange++;
            } else {
                numInRange = 0;
            }
            sleep_ms(instructions->dt*1000);
        }
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
//        calibrator.comm->receive_messages();
//        printf("num_bad_bytes_in %d\n", calibrator.comm->num_bad_bytes_in);
//        if (calibrator.trigger_inbox->get_newest(*calibrator.instructions) >= 0) {
//            printf("Calibrate motor\n");
//            calibrator.calibrate_motor();
//        } else {
            calibrator.instructions->dt = 0.5;
            float angle = calibrator.ADC.read_ADC_raw(calibrator.instructions->motorNum+1);
            float angVel = calibrator.calc_angvel(angle);
            calibrator.report_result(angle, angVel, 0, to_us_since_boot(get_absolute_time()) * 1e-6);
            sleep_ms(500);
//        }
    }
}
