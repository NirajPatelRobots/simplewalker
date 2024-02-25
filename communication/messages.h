/*structs to use in simplewalker communication
TODO: values accessible in multiple ways. bitfields for errcode and command?
*/
#ifndef SIMPLEWALKER_MESSAGES_H
#define SIMPLEWALKER_MESSAGES_H
#include "../mainsrc/walkertypes.h"

//everything in SI units and radians unless stated otherwise

struct MotorCalibrationTriggerMsg { //send to microcontroller to run motor calibration
    uint16_t ID;
    uint16_t motorNum;
    float amplitude;
    float frequency;
    float dt;
    float max_displacement, min_displacement; // max and min angle
    uint16_t send_skip_iterations; // how many interations of counting without sending
    uint16_t text_output; // bool whether to output human-readable stdout or MotorCalibrationStateMsg
};
const uint16_t MotorCalibrationTriggerMsgID = 0x0D11;

struct MotorCalibrationStateMsg { // sent by microcontroller during motor calibration
    uint16_t ID;
    uint32_t timestamp_us;
    float angle;
    float angvel;
    float voltage;
};
const uint16_t MotorCalibrationStateMsgID = 0x0C11;

struct ControlTargetMsg { // target angle and velocity with predicted leg torque from main comp to micro
    uint16_t ID;
    uint16_t command; // can be defined
    uint32_t time_us; //pico clock time to start this command
    float angle[6];
    float angle_vel[6];
    float torque[6];
};
const uint16_t ControlTargetMsgID = 0x0D01;

enum ControlStateError {CTRLSTERR_LOSTCOM = 1, CTRLSTERR_IMU = 0b10, CTRLSTERR_ADC = 0b100, 
                        CTRLSTERR_TEMP = 0b1000 };

struct ControlStateMsg { // sends current motor control and sensor state from the microcontroller
    uint16_t ID;
    uint16_t errcode;
    struct SensorData sensor_data;
};
const uint16_t ControlStateMsgID = 0x0C01;

struct RobotStateMsg {
    uint16_t ID;
    uint16_t errcode;
    uint32_t timestamp_us; //microseconds
    float pos[3];
    float axis[3];
    float vel[3];
    float angvel[3];
    float leg_pos[6];
    float leg_vel[6];
};
const uint16_t RobotStateMsgID = 0x0E01;

#endif
