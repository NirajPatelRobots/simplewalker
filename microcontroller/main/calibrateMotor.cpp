/* Calibrate a motor, following an input and reporting voltage and angle measurements.
*/


#include "../simplewalker_motors.hpp"
#include "../communication/pico_comm.hpp"
#include "../calibrate/motor_calibrator.hpp"
#include <stdio.h>
#include "pico/time.h"


int main() {
    std::unique_ptr<PicoCommunication> comm;
    auto trigger_inbox = make_shared<MessageInbox<MotorCalibrationTriggerMsg>>(MotorCalibrationTriggerMsgID, *comm);
    auto state_outbox = make_shared<MessageOutbox<MotorCalibrationStateMsg>>(MotorCalibrationStateMsgID, *comm);
    MotorCalibrator calibrator{SIMPLEWALKER_MOTOR_IO_SETTINGS, state_outbox,
         ADC_BATTERY_VOLTAGE_CHANNEL, ADC_BATTERY_VOLTAGE_SCALE};
    while (1) {
        comm->receive_messages();
        if (trigger_inbox->get_newest(*calibrator.instructions) >= 0) {
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
