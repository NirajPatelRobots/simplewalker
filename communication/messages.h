/*structs to use in simplewalker communication
TODO: values accessible in multiple ways. bitfields for errcode and command?
*/
#include <stdint.h>

//everything in SI units and radians unless stated otherwise

struct MotorTestCommandMsg { //send to microcontroller to run motor tests
    uint16_t ID;
    uint16_t motorNum;
    float amplitude; //amp and freq are arbitrary and relative
    float frequency;
    float dt;
};
const uint16_t MotorTestCommandMsgID = 0x0D11;

struct ControlTargetMsg { // target angle and velocity with leg torque from comp to micro
    uint16_t ID;
    uint16_t command; // can be defined
    float angle_r[3]; //TODO: accesible through non-right and left 6-arrays?
    float angle_l[3]; //these are targets, not actual
    float angvel_r[3];
    float angvel_l[3];
    float torque_r[3];
    float torque_l[3];
};
const uint16_t ControlTargetMsgID = 0x0D01;

enum ControlStateError {CTRLSTERR_LOSTCOM = 1, CTRLSTERR_IMU = 0b10, CTRLSTERR_ADC = 0b100, 
                        CTRLSTERR_TEMP = 0b1000 };

struct ControlStateMsg { // sends current motor control and sensor state from the microcontroller
    uint16_t ID;
    uint16_t errcode;
    uint32_t timestamp_us; //microseconds
    float accel[3];
    float gyro[3];
    float angle_r[3]; //TODO: accesible through non-right and left 6-arrays?
    float angle_l[3];
    float angvel_r[3];
    float angvel_l[3];
};
const uint16_t ControlStateMsgID = 0x0C01;
