/* tracks target motor path using dynamics to determine motor output commands
TODO:
    negotiates past and future targets
    unit tests
    test motor controller tracking sin wave
    calculates current and predicted state
    slow and stop if message not received
    */
#include "pico/stdlib.h"
#include "micro_parameters.h"


struct MotorControlState {
    uint32_t start_time_us; //controller clock time at the start of this state
    float angle, angvel, torque;
    MotorControlState(uint32_t _start_time_us = 0, float _angle = 0, float _angvel = 0, float _torque = 0)
        : start_time_us(_start_time_us), angle(_angle), angvel(_angvel), torque(_torque) {}
    bool update(uint32_t time_us);
};

class MotorController {
protected:
    MotorControlState target{};
public:
    const int ID;
    MotorController(int motor_ID) : ID(motor_ID) {}
    bool set_target(uint32_t start_time_us, float angle_target, float angvel_target, float torque_pred);
    void update_target(uint32_t time_us) {target.update(time_us);}
    virtual bool calc_command(const MotorControlState &state, float &command) = 0; // returns whether it worked
};

class DCMotorController : public MotorController {
    float K_P, K_V, K_ANGVEL, K_COUL, K_OUT;
public:
    DCMotorController(int ID);
    void set_parameters(float k_p, float k_v, float k_angvel, float k_coul, float k_out);
    bool calc_command(const MotorControlState &state, float &command) override;
};


class ServoMotorController : public MotorController {
    float angle_offset;
public:
    ServoMotorController(int ID) : MotorController(ID), angle_offset(0) {}
    void set_angle_offset(float _angle_offset) {angle_offset = _angle_offset;}
    bool calc_command(const MotorControlState &state, float &command) override;
};
