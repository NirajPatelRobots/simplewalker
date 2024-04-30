/* main thread that does sending and receiving, slow sensor reading for the micro for simplewalker.
Niraj April 2022 */

#include "mpu6050.hpp"
#include "pico_comm.hpp"
#include "../communication/messages.h"
#include "micro_parameters.h"
#include <stdio.h> //DEGUB DEBUG
#include <memory>
using std::shared_ptr, std::make_shared, std::unique_ptr, std::make_unique;


bool handle_IMU(struct repeating_timer *t) {
    ((MPU6050 *)(t->user_data))->read();
    return true;
}

int main() {
    unique_ptr<PicoCommunication> comm{make_unique<PicoCommunication>()};
    auto controlStateOutbox{make_unique<MessageOutbox<ControlStateMsg>>(ControlStateMsgID, *comm)};
    SensorData *sensorData = &controlStateOutbox->message.sensor_data;
    controlStateOutbox->message.ID = ControlStateMsgID;
    MPU6050 *IMU = new MPU6050(sensorData->accel, sensorData->gyro);
    absolute_time_t looptarget;
    struct repeating_timer timer;

    sleep_ms(100);
    IMU->reset();
    IMU->power(1, false, false, false);
    IMU->setscale_accel(1);
    IMU->setscale_gyro(1);
    sleep_ms(100);
    add_repeating_timer_us(-IMU_DT_US, handle_IMU, (void *)IMU, &timer);

    looptarget = get_absolute_time();
    while (1) {
        sensorData->timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time());
        controlStateOutbox->message.errcode |= (CTRLSTERR_IMU + CTRLSTERR_TEMP)
                         * (IMU->chip_temp > IMU_TEMP_MAX);
        controlStateOutbox->send();
        looptarget = delayed_by_us(looptarget, ADMIN_DT_US);
        sleep_until(looptarget);
    }
    delete IMU;
    return 0;
}
