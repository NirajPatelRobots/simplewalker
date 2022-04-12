/* main thread that does sending and receiving, slow sensor reading for the micro for simplewalker.
Niraj April 2022 */

#include "mpu6050.hpp"
#include "pico_comm.hpp"
#include "../communication/messages.h"
#include "micro_parameters.h"
#include <stdio.h> //DEGUB DEBUG


bool handle_IMU(struct repeating_timer *t) {
    ((MPU6050 *)(t->user_data))->read();
    return true;
}

int main() {
    struct ControlStateMsg *state = new struct ControlStateMsg;
    *state = {};
    state->ID = ControlStateMsgID;
    state->errcode = 0;
    MPU6050 *IMU = new MPU6050(state->accel, state->gyro);
    absolute_time_t looptarget;
    struct repeating_timer timer;

    pico_comm_init();
    sleep_ms(100);
    IMU->reset();
    IMU->power(1, false, false, false);
    IMU->setscale_accel(1);
    IMU->setscale_gyro(1);
    sleep_ms(100);
    add_repeating_timer_us(-IMU_DT_US, handle_IMU, (void *)IMU, &timer);

    looptarget = get_absolute_time();
    while (1) {
        state->timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time());
        state->errcode |= (CTRLSTERR_IMU + CTRLSTERR_TEMP)
                         * (IMU->chip_temp > IMU_TEMP_MAX);
        send_struct((char *)state, sizeof(struct ControlStateMsg));
        /*for (int i = 0; i < 12; i++) {
            printf("%.2x|", (uint8_t)(((char *)state)[i]));
        }
        printf("   %f\n", state->accel[0]); */
        looptarget = delayed_by_us(looptarget, ADMIN_DT_US);
        sleep_until(looptarget);
    }
    delete IMU;
    delete state;
    return 0;
}
