#ifndef WALKER_SENSORDATA_H
#define WALKER_SENSORDATA_H
#include <stdint.h>

const int WALKER_COMM_NUM_MOTORS{6};

struct SensorData {
    uint32_t timestamp_us; //microseconds
    float accel[3];
    float gyro[3];
    float angle[WALKER_COMM_NUM_MOTORS];
    float angvel[WALKER_COMM_NUM_MOTORS];
};


#endif