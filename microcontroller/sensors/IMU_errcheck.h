#ifndef SIMPLEWALKER_PICO_IMU_ERRCHECK_H
#define SIMPLEWALKER_PICO_IMU_ERRCHECK_H

#include "../../communication/messages.h"
#include "mpu6050.hpp"

struct IMUOverTempCheck {
    bool is_asleep {false};
    float max_temp;
    explicit IMUOverTempCheck(float max_temp_)
        : max_temp(max_temp_) {}
    // if over temp, toggle MPU6050 sleep mode and return err code
    uint16_t check_temp_err(MPU6050 &IMU);
    ~IMUOverTempCheck();
};

struct IMUScaleCheck {
    float accel_max, gyro_max;
    IMUScaleCheck(MPU6050::Scale accel_scale, MPU6050::Scale gyro_scale) {
        set_limits(accel_scale, gyro_scale);
    }
    void set_limits(MPU6050::Scale accel_scale, MPU6050::Scale gyro_scale, float max_fraction = 0.95);
    void set_msg(IMUDataMsg &IMUMsg) const;
    ~IMUScaleCheck();
};

#endif //SIMPLEWALKER_PICO_IMU_ERRCHECK_H
