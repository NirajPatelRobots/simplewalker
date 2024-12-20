#include <stdint.h>

#ifndef SIMPLEWALKER_PICO_MICRO_PARAMETERS_H
#define SIMPLEWALKER_PICO_MICRO_PARAMETERS_H
#define CONTROL_DT_US 5000
#define ADMIN_DT_US 30000
#define PARAM_KP_DEFAULT 1000.0 // how strongly it's pulled towards reference angle
#define PARAM_KV_DEFAULT 0.0 // how strongly it's pulled by reference velocity
#define PARAM_OUTPUT_DEFAULT 1.0
#define PARAM_ANGVEL_DEFAULT 1.0
#define PARAM_COUL_DEFAULT 1.0
#define IMU_TEMP_MAX 85.0

const uint8_t IMU_DT_MS = 5;
const unsigned IMU_INTROSPECTION_LOOP_MS = 3000;
// filter_cfg (Hz) accel = [260, 184, 94, 44, 21, 10, 5] gyro = [256, 188, 98, 42, 20, 10, 5]
const uint8_t IMU_LP_FILTER_CFG_IDX = 2;  // 96 Hz

const uint32_t COMM_SEM_WAIT_US = 2000;

/* ADC value when V_bat = V_logic: 106
 * V_bat = scale * raw
 * scale = V_logic / ADC_value = 3.22 / 106 */
const float ADC_BATTERY_VOLTAGE_SCALE = 0.0304;
const int ADC_BATTERY_VOLTAGE_CHANNEL = 0;

const float MAX_MOTOR_VOLTAGE = 5.0;

// how quickly (in sec) the controller slows down when it isn't receiving messages (higher=takes longer to stop)
const float CONTROLLER_SPEED_UPDATE_DECAY{1.0};

#endif
