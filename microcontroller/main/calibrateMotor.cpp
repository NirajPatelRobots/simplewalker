/* Calibrate a motor, following an input and reporting voltage and angle measurements.
*/


#include "../simplewalker_motors.hpp"
#include "../communication/pico_comm.hpp"
#include "../calibrate/motor_calibrator.hpp"
#include "../microcontroller/control/motor_model_API.hpp"
#include <stdio.h>
#include "pico/time.h"


void report_state(MotorCalibrationStateMsg &state, MessageOutbox<MotorCalibrationStateMsg> *outbox, bool text_output) {
    if (text_output) {
        printf("%f,%f,%f,%f\n", state.timestamp_us * 1e-6, state.voltage, state.angle, state.reference);
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
    printf("Calibrate motor %d; freq=%f, amp=%f, dt=%f\nAngle filter: despiker=%d, avg=%d",
            calibrator.instructions->motorNum, calibrator.instructions->frequency,
            calibrator.instructions->amplitude, calibrator.instructions->dt,
            VEL_FILT_DESPIKE_N, VEL_FILT_AVG_N);
    if (calibrator.state.voltage < 3.3) {
        printf("Low battery, won't run calibration\n");
    } else {
        calibrator.start_calibration();
    }
}


class MotorModelAPITargetComms {
public:
    shared_ptr<MessageInbox<MotorModelAPI::Request>> model_request_inbox;
    shared_ptr<MessageOutbox<MotorModelAPI::Response>> model_response_outbox;
    MotorModelAPI::Request model_request;
    unsigned num_messages;
    MotorModelAPITargetComms(Communicator *comm) :
        model_request_inbox(make_shared<MessageInbox<MotorModelAPI::Request>>(MotorModelAPIRequestID, *comm)),
        model_response_outbox(make_shared<MessageOutbox<MotorModelAPI::Response>>(MotorModelAPIResponseID, *comm)),
        model_request(), num_messages() {}
    // call after comm.receive_messages()
    void handle(MotorModel &model) {
        if (model_request_inbox->get_newest(model_request) >= 0) {
            model_request_inbox->clear();
            MotorModelAPI::handle_request(model_request, model, model_response_outbox->message);
            model_response_outbox->send();
            num_messages++;
        }
    }
};


int main() {
    auto comm = std::make_unique<PicoCommunication>();
    auto trigger_inbox = make_shared<MessageInbox<MotorCalibrationTriggerMsg>>(MotorCalibrationTriggerMsgID, *comm);
    auto state_outbox = make_shared<MessageOutbox<MotorCalibrationStateMsg>>(MotorCalibrationStateMsgID, *comm);
    MotorModelAPITargetComms motorModelAPIComms{comm.get()};
    MotorCalibrator calibrator{SIMPLEWALKER_MOTOR_IO_SETTINGS};
    calibrator.ADC->set_channel("batteryVoltage", ADC_BATTERY_VOLTAGE_CHANNEL, 0, ADC_BATTERY_VOLTAGE_SCALE);
    absolute_time_t looptarget = get_absolute_time();
    absolute_time_t start_time = looptarget;
    size_t counter = 100;
    while (1) {
        if (calibrator.state.status != MOTORCAL_IDLE) {
            if (!calibrator.run_calibration_loop()) {
                calibrator.motors_IO->set_motor_voltage(calibrator.instructions->motorNum, 0);
                calibrator.state.status = MOTORCAL_IDLE;
                printf("Calibration failed :(\n");
            }
            calibrator.state.timestamp_us = (uint32_t)(looptarget - start_time);
            report_state(calibrator.state, state_outbox.get(), calibrator.instructions->text_output);
            looptarget = delayed_by_us(looptarget, (uint64_t)(calibrator.instructions->dt * 1e6));
            sleep_until(looptarget);
        } else {
            calibrator.state.voltage = calibrator.ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL);
            comm->receive_messages();
            motorModelAPIComms.handle(calibrator.motor_model);
            if (trigger_inbox->get_newest(*calibrator.instructions) >= 0) {
                handle_instructions(calibrator);
                looptarget = get_absolute_time();
                start_time = looptarget;
                counter = 0;
            } else {
                calibrator.motors_IO->set_motor_voltage(calibrator.instructions->motorNum, 0);
                calibrator.read_angle();
                calibrator.state.timestamp_us = (uint32_t)(get_absolute_time() + 1000'000'000);
                if (counter++ <= 20) {
                    report_state(calibrator.state, state_outbox.get(), calibrator.instructions->text_output);
                }
                // printf("num motor model messages: %d\n", motorModelAPIComms.num_messages);
                sleep_ms(500);
            }
        }
    }
}
