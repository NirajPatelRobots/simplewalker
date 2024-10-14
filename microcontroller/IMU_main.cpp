/* main thread that does sending and receiving, slow sensor reading for the micro for simplewalker.
Niraj April 2022 */

#include "mpu6050.hpp"
#include "pico_comm.hpp"
#include "../communication/messages.h"
#include "micro_parameters.h"
#include "pico/binary_info.h"
#include <memory>
using std::unique_ptr, std::make_unique;


const uint8_t IRQ_PIN = 6;
const bool CHECK_OVER_SCALE = true;

static unique_ptr<MPU6050> IMU;
static unique_ptr<PicoCommunication> comm;
static unique_ptr<MessageOutbox<IMUDataMsg>> IMUOutbox;
volatile bool got_irq;


uint16_t check_IMU_over_temp(MPU6050 *IMU) {
    static bool is_asleep = false;
    if (is_asleep) {
        IMU->power(1, false, false, false); // wake up so we can read temp again
        is_asleep = false;
        return IMU_ERR_TEMP;
    } if (IMU->chip_temp > IMU_TEMP_MAX) {
        IMU->power(0, false, true, false);
        is_asleep = true;
        return IMU_ERR_TEMP;
    }
    return IMU_NO_ERR;
}

bool is_over_scale(const float data[3], float limit) {
    for (int i = 0; i < 3; i++) {if (data[i] > limit) return true; }
    return false;
}

void irq_callback(uint gpio, uint32_t events) {
    static const uint32_t DLPF_delay_us = (uint32_t)(IMU->read_timing().accel_timing.delay * 1e3);
    static const float accel_max = MPU6050_max_accel(MPU6050::Scale_0) * 0.95;
    static const float gyro_max = MPU6050_max_gyro_rad(MPU6050::Scale_0) * 0.95;
    IMUOutbox->message.timestamp_us = (uint32_t)to_us_since_boot(get_absolute_time()) - DLPF_delay_us;
    IMU->read();
    IMUOutbox->message.errcode = check_IMU_over_temp(IMU.get());
    if (CHECK_OVER_SCALE)
        IMUOutbox->message.errcode += IMU_ERR_OVER_SCALE * (   is_over_scale(IMUOutbox->message.accel, accel_max)
                                                            || is_over_scale(IMUOutbox->message.gyro,  gyro_max));
    got_irq = true;
    IMUOutbox->send();
}

int main() {
    bi_decl(bi_1pin_with_name(IRQ_PIN, "IMU IRQ pin 1"));
    comm = make_unique<PicoCommunication>();
    IMUOutbox = make_unique<MessageOutbox<IMUDataMsg>>(IMUDataMsgID, *comm);
    IMUOutbox->message.ID = IMUDataMsgID;
    IMU = make_unique<MPU6050>(IMUOutbox->message.accel, IMUOutbox->message.gyro);

    sleep_ms(100);
    IMU->reset();
    IMU->power(1, false, false, false);
    IMU->set_timing(2, IMU_DT_MS - 1); // lowpass = 96 Hz
    IMU->configure_interrupt(false, false, true, true, true);
    sleep_ms(100);
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_LEVEL_HIGH, true, &irq_callback);

    while (true) {
        got_irq = false;
        sleep_ms(IMU_ERROR_CHECK_WAIT_MS);
        if (!got_irq) {
            IMUOutbox->message.errcode = IMU_ERR_NO_IRQ + check_IMU_over_temp(IMU.get())
                                         + (IMU_ERR_NOT_CONNECTED * IMU->is_connected());
            IMUOutbox->send();
        }
    }
    return 0;
}
