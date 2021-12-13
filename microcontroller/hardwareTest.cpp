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
    int repeats = 5, waitms = 300;
    float magnitude = 0.5, angle;
    unsigned int startTime, motorTime, sensorTime;
    while (1) {
        startTime = to_us_since_boot(get_absolute_time());
        motors.setMotor(motorNum, magnitude);
        motorTime = to_us_since_boot(get_absolute_time()) - startTime;
        angle = sensor.readAngle(motorNum);
        sensorTime = to_us_since_boot(get_absolute_time()) - motorTime - startTime;
        printf("Angle: %3f , Motor set time: %d us, Angle read time: %d us\n", angle, motorTime, sensorTime);
        sleep_ms(waitms);

        startTime = to_us_since_boot(get_absolute_time());
        motors.setMotor(motorNum, -magnitude);
        motorTime = to_us_since_boot(get_absolute_time()) - startTime;
        angle = sensor.readAngle(motorNum);
        sensorTime = to_us_since_boot(get_absolute_time()) - motorTime - startTime;
        printf("Angle: %3f , Motor set time: %d us, Angle read time: %d us\n", angle, motorTime, sensorTime);
        sleep_ms(waitms);
    }
}

