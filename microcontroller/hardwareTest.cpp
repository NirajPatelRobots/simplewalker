//test motors.cpp and ADCReader.cpp
#include "micro_parameters.h"
#include "simplewalker_motors.hpp"
#include "ADC_reader.hpp"
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    Simplewalker_MotorOutput motorOutput;
    Motornum motorNum = right_hip;
    ADCReader ADC{};
    shared_ptr<ADCChannel> batteryVoltage = ADC.set_channel("batteryVoltage", 0, 0, ADC_BATTERY_VOLTAGE_SCALE);
    shared_ptr<ADCChannel> right_hip_sensor = ADC.set_channel("right hip motor", 1, ADC_ANGLE_OFFSET, ADC_ANGLE_SCALE);
    ADC.connect_SPI();
    int waitms = 1000;
    float magnitude{-0.5}, angle;
    unsigned int startTime, motorTime, ADCtime, multTime, sendTime = 0;
    while (1) {
        startTime = to_us_since_boot(get_absolute_time());
        motorOutput.set(motorNum, magnitude);
        motorTime = to_us_since_boot(get_absolute_time()) - startTime;
        angle = ADC.read_ADC_scaled(1);
        ADC.read_ADC_scaled(0);
        ADCtime = to_us_since_boot(get_absolute_time()) - motorTime - startTime;
        magnitude = -magnitude;
        multTime = to_us_since_boot(get_absolute_time()) - ADCtime - motorTime - startTime;
        printf("Batt: %.1f V, Angle: %.2f, time[us]:(Motor: %d, ADC: %d, Mult: %d, Send: %d)\n",
               batteryVoltage->scaled_value, angle, motorTime, ADCtime, multTime, sendTime);
        sendTime = to_us_since_boot(get_absolute_time()) - multTime - ADCtime - motorTime - startTime;
        sleep_ms(waitms);
    }
}
