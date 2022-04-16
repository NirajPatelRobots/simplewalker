#include "sensorBoss.hpp"

const size_t SensorBoss::IDX_ACCEL = 0;
const size_t SensorBoss::IDX_GYRO = 3;

const Vector3f gravity(0, 0, -1);


SensorBoss::SensorBoss(const float *msgsensordata, float accel_stddev, float gyro_stddev) 
: data(msgsensordata, M), prediction(sens_pred), covariance(cov), jacobian(jac) {
    //data = Eigen::Map<const VectorXf>(msgsensordata, M);
    sens_pred = VectorXf::Zero(M);
    ///cov.setIdentity();
    cov = MatrixXf::Identity(M, M);
    //cov(Eigen::seqN(Eigen::fix<IDX_ACCEL>, Eigen::fix<3>)) *= accel_stddev * accel_stddev;
    //cov(Eigen::seqN(Eigen::fix<IDX_GYRO>, Eigen::fix<3>)) *= gyro_stddev * gyro_stddev;
    cov.block<3,3>(IDX_ACCEL, IDX_ACCEL) *= accel_stddev * accel_stddev;
    cov.block<3,3>(IDX_GYRO, IDX_GYRO) *= gyro_stddev * gyro_stddev;
    jac = MatrixXf::Zero(M, N);
}


void SensorBoss::predict(const RobotState &state_pred, const RobotState &last_state, float dt, const Matrix3f *Jac_RT_Eul) {
    sens_pred.segment<3>(IDX_ACCEL) = state_pred.RT * (state_pred.vel() - last_state.vel() + gravity) / dt;
    jac.block<3,3>(IDX_ACCEL, RobotState::IDX_VEL) = state_pred.RT / dt;
    for (int i=0; i<3; i++) {
        jac.block<3,1>(IDX_ACCEL, RobotState::IDX_EUL + i) = Jac_RT_Eul[i] * (state_pred.vel() - last_state.vel() + gravity) / dt;
    }
    sens_pred.segment<3>(IDX_GYRO) = state_pred.RT * state_pred.angvel();
    jac.block<3,3>(IDX_GYRO, RobotState::IDX_ANGVEL) = state_pred.RT;
    for (int i=0; i<3; i++) {
        jac.block<3,1>(IDX_GYRO, RobotState::IDX_EUL + i) = Jac_RT_Eul[i] * state_pred.angvel();
    }
}

const Vector3f SensorBoss::accel() const {return data.segment<3>(IDX_ACCEL);}
const Vector3f SensorBoss::gyro() const {return data.segment<3>(IDX_GYRO);}

