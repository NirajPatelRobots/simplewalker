/* main thread that does sensor reading, sending and receiving for the micro for simplewalker.
Niraj Feb 2024
TODO:
    class for angle reading, angvel filter, set sensorData and MotorControlState
    move log timing stuff somewhere else
    class for running state, with other classes as members, abstracts NUM_MOTORS loops
        shouldn't include loop timing nor communication
 */

#include "pico_comm.hpp"
#include "../communication/messages.h"
#include "micro_parameters.h"
#include "simplewalker_motors.hpp"
#include "motor_control.hpp"
#include <stdio.h>
#include <memory>
#include <chrono>
using std::shared_ptr, std::make_shared, std::unique_ptr, std::make_unique;

uint16_t next_logtime() {
    namespace chrono = std::chrono;
    static chrono::time_point<chrono::steady_clock> timerstart;
    chrono::time_point<chrono::steady_clock> now = chrono::steady_clock::now();
    auto logtime = (uint16_t)(chrono::duration<float, std::micro>(now - timerstart).count());
    timerstart = now;
    return logtime;
}

int main() {
    unique_ptr<PicoCommunication> comm{make_unique<PicoCommunication>()};
    auto controlStateOutbox{make_unique<MessageOutbox<ControlStateMsg>>(ControlStateMsgID, *comm)};
    auto controlInfoOutbox{make_unique<MessageOutbox<ControlInfoMsg>>(ControlInfoMsgID, *comm)};
    shared_ptr<ADCReader> ADC{make_shared<ADCReader>()};
    shared_ptr<ADCChannel> batteryVoltage = ADC->set_channel(
            "batteryVoltage", ADC_BATTERY_VOLTAGE_CHANNEL, 0, ADC_BATTERY_VOLTAGE_SCALE);
    std::unique_ptr<MotorsIO> motors_IO{std::make_unique<MotorsIO>(SIMPLEWALKER_MOTOR_IO_SETTINGS, ADC)};
    auto target_inbox{make_shared<MessageInbox<ControlTargetMsg>>(ControlTargetMsgID, *comm)};
    shared_ptr<ControlTargetMsg> targetMsg{make_shared<ControlTargetMsg>()};
    float last_angles[NUM_MOTORS] = {0};
    bool target_msg_recent{false};
    unsigned controller_info_counter{0};

    std::vector<DCMotorController> controllers{};
    for (int i{}; i < NUM_MOTORS; i++) {
        controllers.emplace_back(i);
        controllers.at(i).set_parameters(PARAM_KP_DEFAULT, PARAM_KV_DEFAULT, PARAM_ANGVEL_DEFAULT,
                                         PARAM_COUL_DEFAULT, PARAM_OUTPUT_DEFAULT);
    }

    ADC->connect_SPI();
    motors_IO->initialize_ADC_channels();
    SensorData *sensorData = &controlStateOutbox->message.sensor_data;
    controlStateOutbox->message.ID = ControlStateMsgID;
    controlInfoOutbox->message.ID = ControlInfoMsgID;
    absolute_time_t looptarget;
    uint16_t info_tx_time = 0;

    looptarget = get_absolute_time();
    while (1) {
        sensorData->timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time());
        motors_IO->set_battery_voltage(ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL));
        controlInfoOutbox->message.ADC_time_us = next_logtime();
        comm->receive_messages();
        target_msg_recent = (target_inbox->get_newest(*targetMsg) >= 0);
        controlInfoOutbox->message.comm_rx_time_us = next_logtime();
        controlInfoOutbox->message.motor_set_time_us = 0;
        for (int i{}; i < NUM_MOTORS; i++) {
            if (target_msg_recent) {
                controllers[i].set_target(targetMsg->start_time_us,
                                          targetMsg->angle[i], targetMsg->angle_vel[i], targetMsg->torque[i]);
            } else {
                controllers[i].update_target(sensorData->timestamp_us);
                controlStateOutbox->message.errcode |= CTRLSTERR_LOSTCOM;
            }
            float angle = ADC->read_ADC_scaled(SIMPLEWALKER_MOTOR_IO_SETTINGS[i].sensor_channel_num);
            controlInfoOutbox->message.ADC_time_us += next_logtime();
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
            controlInfoOutbox->message.motor_set_time_us += next_logtime();
            last_angles[i] = angle;
        }
        controlStateOutbox->send();
        controlInfoOutbox->message.comm_tx_time_us = next_logtime();
        if (++controller_info_counter == 100) {
            controller_info_counter = 0;
            controlInfoOutbox->message.timestamp_us = sensorData->timestamp_us;
            controlInfoOutbox->message.battery_voltage = motors_IO->get_battery_voltage();
            controlInfoOutbox->message.comm_tx_time_us += info_tx_time;
            controlInfoOutbox->send();
            info_tx_time = next_logtime();
            controlStateOutbox->message.errcode = 0;
        }
        looptarget = delayed_by_us(looptarget, ADMIN_DT_US);
        sleep_until(looptarget);
        controlInfoOutbox->message.sleep_time_us = next_logtime();
    }
    return 0;
}
