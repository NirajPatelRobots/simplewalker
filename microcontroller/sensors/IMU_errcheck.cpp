#include "IMU_errcheck.h"

uint16_t IMUOverTempCheck::check_temp_err(MPU6050 &IMU) {
    if (is_asleep) {
        IMU.power(1, false, false, false); // wake up so we can read temp again
        is_asleep = false;
        return IMU_ERR_TEMP;
    } if (IMU.chip_temp > max_temp) {
        IMU.power(0, false, true, false);
        is_asleep = true;
        return IMU_ERR_TEMP;
    }
    return IMU_NO_ERR;
}

IMUOverTempCheck::~IMUOverTempCheck() = default;

bool is_over_scale(const float data[3], float limit) {
    for (int i = 0; i < 3; i++) {if (data[i] > limit) return true; }
    return false;
}

void IMUScaleCheck::set_limits(MPU6050::Scale accel_scale, MPU6050::Scale gyro_scale, float max_fraction) {
    accel_max = MPU6050_max_accel(accel_scale) * max_fraction;
    gyro_max = MPU6050_max_accel(gyro_scale) * max_fraction;
}

void IMUScaleCheck::set_msg(IMUDataMsg &IMUMsg) const {
    IMUMsg.errcode |= IMU_ERR_OVER_SCALE * (   is_over_scale(IMUMsg.accel, accel_max)
                                            || is_over_scale(IMUMsg.gyro,  gyro_max));
}

IMUScaleCheck::~IMUScaleCheck() = default;
