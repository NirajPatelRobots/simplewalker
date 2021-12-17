//test motors.cpp and sensorReader.cpp
#include "motors.hpp"
#include "sensorReader.hpp"
#include <stdio.h>
#include "pico/stdlib.h"

int main() {
    stdio_init_all();
    Motors motors;
    Motornum motorNum = right_hip;
    SensorReader sensor;
    int repeats = 5, waitms = 1000;
    float magnitude = 0.5, angle, angle2;
    unsigned int startTime, motorTime, sensorTime, multTime, sendTime = 0;
    while (1) {
        startTime = to_us_since_boot(get_absolute_time());
        motors.setMotor(motorNum, magnitude);
        motorTime = to_us_since_boot(get_absolute_time()) - startTime;
        angle = sensor.readAngle(motorNum);
        sensorTime = to_us_since_boot(get_absolute_time()) - motorTime - startTime;
        for (int i = 0; i < 100; i++) {
            angle2 = angle * i;
        }
        multTime = to_us_since_boot(get_absolute_time()) - sensorTime - motorTime - startTime;
        printf("Angle: %.2f, Motortime: %d, Angletime: %d, Multiplytime: %d, Sendtime: %d us\n",
               angle, motorTime, sensorTime, multTime, sendTime);
        sendTime = to_us_since_boot(get_absolute_time()) - multTime - sensorTime - motorTime - startTime;
        sleep_ms(waitms);
        magnitude = -magnitude;
    }
}
