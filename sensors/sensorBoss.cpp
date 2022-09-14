#include "sensorBoss.hpp"

const size_t SensorBoss::IDX_ACCEL = 0;
const size_t SensorBoss::IDX_GYRO = 3;

SensorBoss::SensorBoss(const float *msgsensordata, float accel_stddev, float gyro_stddev)
: vect_pred(VectorXf::Zero(M)), cov(MatrixXf::Identity(M, M)), jac(MatrixXf::Zero(M, N)),
 data_vect(msgsensordata, M), prediction(vect_pred), covariance(cov), jacobian(jac),
 raw_data((SensorData*)msgsensordata), data_pred((SensorData*)&vect_pred(0)) {
    cov.block<3,3>(IDX_ACCEL, IDX_ACCEL) *= accel_stddev * accel_stddev;
    cov.block<3,3>(IDX_GYRO, IDX_GYRO) *= gyro_stddev * gyro_stddev;
}


void SensorBoss::predict(const RobotState &state_pred, const RobotState &last_state, float dt) {
    Vector3f accel_pred = (state_pred.vel() - last_state.vel()) / dt + IMU_GRAVITY;
    vect_pred.segment<3>(IDX_ACCEL) = state_pred.RT * accel_pred;
    jac.block<3,3>(IDX_ACCEL, RobotState::IDX_VEL) = state_pred.RT / dt;
    Jac_rotated_wrt_axis_angle(jac.block<3,3>(IDX_ACCEL, RobotState::IDX_AXIS), state_pred.axis(), accel_pred);

    vect_pred.segment<3>(IDX_GYRO) = state_pred.RT * state_pred.angvel();
    jac.block<3,3>(IDX_GYRO, RobotState::IDX_ANGVEL) = state_pred.RT;
    Jac_rotated_wrt_axis_angle(jac.block<3,3>(IDX_GYRO, RobotState::IDX_AXIS), state_pred.axis(), state_pred.angvel());
}

void SensorBoss::incrementdata(int numtoskip) {
    new (&data_vect) Eigen::Map<const VectorXf>(data_vect.data() + M + numtoskip, M);
}

bool SensorBoss::data_is_valid(void) const {
    return true;    //TODO
}

const Vector3f SensorBoss::accel() const {return data_vect.segment<3>(IDX_ACCEL);}
const Vector3f SensorBoss::gyro() const {return data_vect.segment<3>(IDX_GYRO);}

const Vector3f SensorBoss::accel_pred() const {return prediction.segment<3>(IDX_ACCEL);}
const Vector3f SensorBoss::gyro_pred() const {return prediction.segment<3>(IDX_GYRO);}
