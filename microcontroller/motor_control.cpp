#include "motor_control.hpp"


bool MotorControlState::update(uint32_t time_us) {
    if (time_us <= start_time_us) return false;
    angle += angvel * 1e6 * (time_us - start_time_us);
    return true;
}

bool MotorController::set_target(uint32_t start_time_us, float angle_target, float angvel_target, float torque_pred) {
    if (start_time_us <= target.start_time_us) return false;
    target.start_time_us = start_time_us;
    target.angle = angle_target;
    target.angvel = angvel_target;
    target.torque = torque_pred;
}


DCMotorController::DCMotorController(int ID)
        : MotorController(ID), K_P(PARAM_KP_DEFAULT), K_V(PARAM_KV_DEFAULT),
          K_ANGVEL(PARAM_ANGVEL_DEFAULT), K_COUL(PARAM_COUL_DEFAULT), K_OUT(PARAM_OUTPUT_DEFAULT) {}

void DCMotorController::set_parameters(float k_p, float k_v, float k_angvel, float k_coul, float k_out) {
    K_P = k_p;    K_V = k_v;    K_ANGVEL = k_angvel;    K_COUL = k_coul;    K_OUT = k_out;
}

bool DCMotorController::calc_command(const MotorControlState &state, float &command) {
    if (!target.update(state.start_time_us)) return false;
    float ang_accel_ref = K_P * (target.angle - state.angle) + K_V * (target.angvel - state.angvel);
    command = (ang_accel_ref - K_ANGVEL * state.angvel - target.torque
               - K_COUL * (state.angvel > 0 ? 1.0 : -1.0))            / K_OUT;
    return true;
}


bool ServoMotorController::calc_command(const MotorControlState &state, float &command) {
    if (!target.update(state.start_time_us)) return false;
    command = target.angle + angle_offset;
    return true;
}

/*
class AllMotorController {
    struct ReferenceMsg ref;
    float angle[NUM_MOTORS], angle_vel[NUM_MOTORS], angle_vel_filt[NUM_MOTORS], V_battery;
    uint32_t time_us, dt_us, last_msg_time_us;
public:
    MotorController(void);
    void set_reference(const struct ReferenceMsg &new_ref) {
        ref = new_ref;
    };
    void get_state(struct MotorState &state) {
        for (int i =0; i < sizeof(struct MotorState); i++) {
            state.angle[i] = angle[i];
            state.angle_vel[i] = angle_vel[i];
        }
        state.V_battery = V_battery;
        state.time_us = time_us;
    }
    void main(void) {
        V_battery = sensors.readBatteryVoltage();
        uint32_t new_time = (uint32_t)to_us_since_boot(get_absolute_time());
        dt_us = (new_time - time_us);
        time_us = new_time;
        for (int i = 0; i < NUM_MOTORS; i++) {
            calc_joint(i);
            ref.angle[i] += ref.angle_vel[i] * 1e6 / dt_us; //update reference angle as we move
        }
    };
    void calc_joint(int num) {
        new_angle = sensors.readAngle(num);
        angle_vel[num] = (new_angle - angle[num]) * 1e6 / dt_us;
        angle[num] = new_angle;
        angle_vel_filt[num] = 0.8 * angle_vel[num] + 0.2 * angle_vel_filt[num];

        float ang_accel_ref = PARAM_KP * (ref.angle[num] - angle[num]) + PARAM_KV * (ref.angle_vel[num] - angle_vel[num]);
        float voltage = (ang_accel_ref - PARAM_ANGVEL * angle_vel[num] - ref.torque[num]
                         - PARAM_COUL * (angle_vel[num] > 0 ? 1.0 : -1.0)) / PARAM_OUTPUT;
        motors.setMotor(num, voltage / V_battery);
    };
};
*/
