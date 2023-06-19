#define CONTROL_DT_US 5000
#define ADMIN_DT_US 30000
#define IMU_DT_US 5000
#define PARAM_KP_DEFAULT 1000.0 // how strongly it's pulled towards reference angle
#define PARAM_KV_DEFAULT 0.0 // how strongly it's pulled by reference velocity
#define PARAM_OUTPUT_DEFAULT 1.0
#define PARAM_ANGVEL_DEFAULT 1.0
#define PARAM_COUL_DEFAULT 1.0
#define IMU_TEMP_MAX 85.0

/* R_high = 329 kOhm R_low = 993 kOhm
O-------/\/\/\/---o----/\/\/\/----O
V_Batt  R_high  sensor  R_low   GND
V_sens = V_Batt*R_low/(R_high+R_low)
scale = V_ref/V_sens/max_sense = 3.3/0.249/1023 */
const float ADC_BATTERY_VOLTAGE_SCALE = 0.0130;
const int ADC_ANGLE_OFFSET = -512;
const float ADC_ANGLE_SCALE = 0.02;
