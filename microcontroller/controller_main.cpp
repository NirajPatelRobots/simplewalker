/* main thread that does sensor reading, sending and receiving for the micro for simplewalker.
Niraj Feb 2024
TODO:
    receive target state
    class for angle reading, angvel filter, set sensorData and MotorControlState
    class for running state, with other classes as members, abstracts NUM_MOTORS loops
        shouldn't include not loop timing, nor communication
    separate messages for IMU data and angle data?
 */

#include "pico_comm.hpp"
#include "../communication/messages.h"
#include "micro_parameters.h"
#include "simplewalker_motors.hpp"
#include "motor_control.hpp"
#include <stdio.h>
#include <memory>
using std::shared_ptr, std::make_shared, std::unique_ptr, std::make_unique;


int main() {
    unique_ptr<PicoCommunication> comm{make_unique<PicoCommunication>()};
    auto controlStateOutbox{make_unique<MessageOutbox<ControlStateMsg>>(ControlStateMsgID, *comm)};
    shared_ptr<ADCReader> ADC{make_shared<ADCReader>()};
    shared_ptr<ADCChannel> batteryVoltage = ADC->set_channel(
            "batteryVoltage", ADC_BATTERY_VOLTAGE_CHANNEL, 0, ADC_BATTERY_VOLTAGE_SCALE);
    std::unique_ptr<MotorsIO> motors_IO{std::make_unique<MotorsIO>(SIMPLEWALKER_MOTOR_IO_SETTINGS, ADC)};
    auto target_inbox{make_shared<MessageInbox<ControlTargetMsg>>(ControlTargetMsgID, *comm)};
    shared_ptr<ControlTargetMsg> targetMsg{make_shared<ControlTargetMsg>()};
    float last_angles[NUM_MOTORS] = {0};
    bool target_msg_recent{false};

    std::vector<DCMotorController> controllers{};
    for (int i{}; i < NUM_MOTORS; i++) {
        controllers.push_back(i);
        controllers.at(i).set_parameters(PARAM_KP_DEFAULT, PARAM_KV_DEFAULT, PARAM_ANGVEL_DEFAULT,
                                         PARAM_COUL_DEFAULT, PARAM_OUTPUT_DEFAULT);
    }

    ADC->connect_SPI();
    motors_IO->initialize_ADC_channels();
    SensorData *sensorData = &controlStateOutbox->message.sensor_data;
    controlStateOutbox->message.ID = ControlStateMsgID;
    absolute_time_t looptarget;

    looptarget = get_absolute_time();
    while (1) {
        sensorData->timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time());
        motors_IO->set_battery_voltage(ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL));
        comm->receive_messages();
        target_msg_recent = (target_inbox->get_newest(*targetMsg) >= 0);
        for (int i{}; i < NUM_MOTORS; i++) {
            if (target_msg_recent) {
                controllers[i].set_target(targetMsg->start_time_us,
                                          targetMsg->angle[i], targetMsg->angle_vel[i], targetMsg->torque[i]);
            } else {
                controllers[i].update_target(sensorData->timestamp_us);
            }
            float angle = ADC->read_ADC_scaled(SIMPLEWALKER_MOTOR_IO_SETTINGS[i].sensor_channel_num);
            float angvel = (angle - last_angles[i]) * ADMIN_DT_US * 1e6;
            sensorData->angle[i] = angle;
            sensorData->angvel[i] = angvel;
            MotorControlState state(sensorData->timestamp_us, angle, angvel, targetMsg->torque[i]);
            float motor_voltage;
            if (controllers[i].calc_command(state, motor_voltage)) {
                motors_IO->set_motor_voltage(i, motor_voltage);
            } else {
                motors_IO->set_motor_voltage(i, 0.0);
            }
            last_angles[i] = angle;
        }
        controlStateOutbox->send();
        looptarget = delayed_by_us(looptarget, ADMIN_DT_US);
        sleep_until(looptarget);
    }
    return 0;
}
