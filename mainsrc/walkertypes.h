#ifndef WALKERTYPES_H
#define WALKERTYPES_H
#include <stdint.h>

struct SensorData {
    uint32_t timestamp_us; //microseconds
    float accel[3];
    float gyro[3];
    float angle_r[3]; //TODO: accesible through non-right and left 6-arrays?
    float angle_l[3];
    float angvel_r[3];
    float angvel_l[3];
};


#endif