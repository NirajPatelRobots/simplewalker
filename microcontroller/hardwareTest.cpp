//test motor reading and setting
#include "micro_parameters.h"
#include "simplewalker_motors.hpp"
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    int motorNum = (int)Motornum::right_hip;
    shared_ptr<ADCReader> ADC{make_shared<ADCReader>()};
    shared_ptr<ADCChannel> batteryVoltage = ADC->set_channel(
            "batteryVoltage", ADC_BATTERY_VOLTAGE_CHANNEL, 0, ADC_BATTERY_VOLTAGE_SCALE);
    std::unique_ptr<MotorsIO> motors_IO{std::make_unique<MotorsIO>(SIMPLEWALKER_MOTOR_IO_SETTINGS, ADC)};
    ADC->connect_SPI();
    motors_IO->initialize_ADC_channels();
    int waitms = 2000;
    float magnitude{1}, angle;
    unsigned int startTime, motorTime, ADCtime, multTime, sendTime = 0;
    while (1) {
        startTime = to_us_since_boot(get_absolute_time());
        motors_IO->set_battery_voltage(ADC->read_ADC_scaled(ADC_BATTERY_VOLTAGE_CHANNEL));
        angle = ADC->read_ADC_scaled(SIMPLEWALKER_MOTOR_IO_SETTINGS[motorNum].sensor_channel_num);
        ADCtime = to_us_since_boot(get_absolute_time()) - startTime;
        motors_IO->set_motor_voltage(motorNum, magnitude);
        motorTime = to_us_since_boot(get_absolute_time()) - ADCtime - startTime;
        magnitude = -magnitude;
        multTime = to_us_since_boot(get_absolute_time()) - ADCtime - motorTime - startTime;
        printf("VBatt=%.2f, Angle=%.2f, time[us]:(Motor: %d, ADC: %d, Mult: %d, Send: %d)\n",
               batteryVoltage->scaled_value, angle, motorTime, ADCtime, multTime, sendTime);
        sendTime = to_us_since_boot(get_absolute_time()) - multTime - ADCtime - motorTime - startTime;
        sleep_ms(waitms);
    }
}
