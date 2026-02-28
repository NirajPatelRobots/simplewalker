#include "controller_info.hpp"

#include <malloc.h>
//#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"

// thank you Silverlock for https://forums.raspberrypi.com/viewtopic.php?t=347638#p2082565
uint32_t getTotalHeap() {
    extern char __StackLimit, __bss_end__;
    return &__StackLimit  - &__bss_end__;
}

uint32_t getFreeHeapSize() {
    struct mallinfo m = mallinfo();
    return getTotalHeap() - m.uordblks;
}

uint32_t getUsedHeapSize() {
    struct mallinfo m = mallinfo();
    return m.uordblks;
}

void init_controller_temp_sensor() {
    adc_init();
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);
}

/* Copied from pico-examples onboard_temperature.c */
float read_onboard_temperature() {
    /* 12-bit conversion, assume max value == ADC_VREF == 3.3 V */
    const float conversionFactor = 3.3f / (1 << 12);

    float adc = (float)adc_read() * conversionFactor;
    float tempC = 27.0f - (adc - 0.706f) / 0.001721f;

    return tempC;
}


// set msg fields (other than ID)
void set_controller_info_msg(struct ControllerInfoMsg &msg) {
    msg.timestamp_us = time_us_64();
    msg.free_heap_bytes = getFreeHeapSize();
    msg.processor_temp = read_onboard_temperature();
}

MessageOutbox<ControllerInfoMsg> create_controller_info_outbox(Communicator &comm) {
    init_controller_temp_sensor();
    MessageOutbox<ControllerInfoMsg> controllerInfoOutbox(ControllerInfoMsgID, comm);
    controllerInfoOutbox.message.ID = ControllerInfoMsgID;
    return controllerInfoOutbox;
}

void set_send_controller_info(MessageOutbox<ControllerInfoMsg> &controllerInfoOutbox) {
    set_controller_info_msg(controllerInfoOutbox.message);
    controllerInfoOutbox.send();
}
