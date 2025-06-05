/* Simplewalker with IMU only
 * For IMU calibration or debugging
Niraj April 2022 */

#include "pico_comm.hpp"
#include "micro_parameters.h"
#include "pico/binary_info.h"
#include "pico/sem.h"
#include "hardware/gpio.h"
#include "sensors/IMUContext.h"


const uint8_t IRQ_PIN = 6;
volatile bool got_irq;
unique_ptr<IMUContext> IMUState;


void irq_callback(uint, uint32_t) {
    IMUState->read_and_send();
    got_irq = true;
}

int main() {
    bi_decl(bi_1pin_with_name(IRQ_PIN, "IMU IRQ pin 1"));
    auto comm = make_unique<PicoCommunication>();
    auto sem = make_shared<semaphore_t>();
    sem_init(sem.get(), 1, 1);
    IMUState = make_unique<IMUContext>(*comm, sem);

    sleep_ms(500);
    IMUState->setup_IMU();
    sleep_ms(100);
    gpio_set_irq_enabled_with_callback(IRQ_PIN, GPIO_IRQ_LEVEL_HIGH, true, &irq_callback);

    while (true) {
        got_irq = false;
        sleep_ms(IMU_INTROSPECTION_LOOP_MS);
        IMUState->send_IMU_info();
        if (!got_irq) {
            IMUState->send_IMUdata_missing(true);
        }
    }
    return 0;
}
