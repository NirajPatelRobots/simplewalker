/* Calibrate a motor, following an input and reporting voltage and angle measurements.
*/


#include "../simplewalker_motors.hpp"
#include "../communication/pico_comm.hpp"
#include "../calibrate/motor_calibrator.hpp"
#include <stdio.h>
#include "pico/time.h"


void report_state(MotorCalibrationStateMsg &state, MessageOutbox<MotorCalibrationStateMsg> *outbox, bool text_output) {
    if (text_output) {
        printf("%f,%f,%f\n", state.timestamp_us * 1e-6, state.voltage, state.angle);
    } else {
        outbox->message = state;
        outbox->send();
    }
}


void handle_instructions(MotorCalibrator &calibrator) {
    if (calibrator.instructions->motorNum >= SIMPLEWALKER_MOTOR_IO_SETTINGS.size()) {
        printf("Motor number doesn't exist, using motor 0\n");
        calibrator.instructions->motorNum = 0;
    }
    printf("Calibrate motor %d; freq=%f, amp=%f, dt=%f\n",
            calibrator.instructions->motorNum, calibrator.instructions->frequency,
            calibrator.instructions->amplitude, calibrator.instructions->dt);
    if (calibrator.state.voltage < 3.3) {
        printf("Low battery, won't run calibration\n");
    } else {
        if (calibrator.instructions->text_output) {
            printf("Time,Voltage,Angle\n");
        }
        calibrator.start_calibration();
    }
}


int main() {
    auto comm = std::make_unique<PicoCommunication>();
    auto trigger_inbox = make_shared<MessageInbox<MotorCalibrationTriggerMsg>>(MotorCalibrationTriggerMsgID, *comm);
    auto state_outbox = make_shared<MessageOutbox<MotorCalibrationStateMsg>>(MotorCalibrationStateMsgID, *comm);
    MotorCalibrator calibrator{SIMPLEWALKER_MOTOR_IO_SETTINGS,
         ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_VOLTAGE_SCALE};
    absolute_time_t looptarget = get_absolute_time();
    absolute_time_t start_time = looptarget;
    while (1) {
        if (calibrator.state.status != MOTORCAL_IDLE) {
            if (!calibrator.run_calibration_loop()) {
                calibrator.motors_IO->set_motor_voltage(calibrator.instructions->motorNum, 0);
                calibrator.state.status = MOTORCAL_IDLE;
            }
            calibrator.state.timestamp_us = (uint32_t)(looptarget - start_time);
            report_state(calibrator.state, state_outbox.get(), calibrator.instructions->text_output);
            looptarget = delayed_by_us(looptarget, (uint64_t)(calibrator.instructions->dt * 1e6));
            sleep_until(looptarget);
        } else {
            calibrator.state.voltage = calibrator.ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL);
            comm->receive_messages();
            if (trigger_inbox->get_newest(*calibrator.instructions) >= 0) {
                handle_instructions(calibrator);
                start_time = looptarget;
            } else {
                calibrator.motors_IO->set_motor_voltage(calibrator.instructions->motorNum, 0);
                calibrator.read_angle();
                calibrator.state.timestamp_us = (uint32_t)(looptarget);
                report_state(calibrator.state, state_outbox.get(), calibrator.instructions->text_output);
                sleep_ms(500);
            }
        }
    }
}
