/* state estimation without legs
    March 2022
*/
#include "state_estimation.hpp"
#include <cmath>

#define YI_ACCEL 0
#define YI_GYRO 3
const int M = 6;
const Vector3f gravity(0, 0, -1);

StateEstimator::StateEstimator(float accel_stddev, float gyro_stddev, float timestep,
                                float pos_stddev, float euler_stddev, float vel_stddev, float angvel_stddev)
    : sensor_prediction(sens_pred), state_covariance(cov), dt(timestep) {
    sensordata = VectorXf::Zero(M);
    cov = 0.01 * MatrixXf::Identity(N, N);
    sens_pred = VectorXf::Zero(M);
    sens_cov = MatrixXf::Identity(M, M) * dt;
    sens_cov.block<3,3>(YI_ACCEL, YI_ACCEL) *= pow(accel_stddev, 2);
    sens_cov.block<3,3>(YI_GYRO, YI_GYRO) *= pow(gyro_stddev, 2);
    for (int i = 0; i < 3; i++) {
        Jac_RT_Eul[i] = Matrix3f::Zero();
    }
    motion_noise = MatrixXf::Identity(N, N) * dt;
    motion_noise.block<3,3>(RobotState::IDX_POS, RobotState::IDX_POS) *= pow(pos_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_EUL, RobotState::IDX_EUL) *= pow(euler_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_VEL, RobotState::IDX_VEL) *= pow(vel_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_ANGVEL, RobotState::IDX_ANGVEL) *= pow(angvel_stddev, 2);
    // set sens_jac and motion_jac from models
    motion_jac = MatrixXf::Identity(N, N);
    motion_jac.block<3,3>(RobotState::IDX_POS, RobotState::IDX_VEL) = Matrix3f::Identity() * dt;
    motion_jac.block<3,3>(RobotState::IDX_EUL, RobotState::IDX_ANGVEL) = Matrix3f::Identity() * dt;
    sens_jac = MatrixXf::Zero(M, N);
}

void StateEstimator::predict() {
    //set state_pred from state (motion model)
    state_pred.pos() = state.pos() + state.vel() * dt;
    state_pred.euler() = state.euler() + state.angvel() * dt;
    state_pred.vel() = state.vel() * (1-dt); // DEBUG DEBUG TODO REMOVE DAMPING
    state_pred.angvel() = state.angvel();
    state_pred.calculate(Jac_RT_Eul);
    cov_pred = motion_jac * cov * motion_jac.transpose() + motion_noise;
    //set sens_pred and sens_jac from state_pred (sensor model)
    sens_pred.segment<3>(YI_ACCEL) = state_pred.RT * (state_pred.vel() - state.vel() + gravity) / dt;
    sens_jac.block<3,3>(YI_ACCEL, RobotState::IDX_VEL) = state_pred.RT / dt;
    for (int i=0; i<3; i++) {
        sens_jac.block<3,1>(YI_ACCEL, RobotState::IDX_EUL + i) = Jac_RT_Eul[i] * (state_pred.vel() - state.vel() + gravity) / dt;
    }
    sens_pred.segment<3>(YI_GYRO) = state_pred.RT * state_pred.angvel();
    sens_jac.block<3,3>(YI_GYRO, RobotState::IDX_ANGVEL) = state_pred.RT;
    for (int i=0; i<3; i++) {
        sens_jac.block<3,1>(YI_GYRO, RobotState::IDX_EUL + i) = Jac_RT_Eul[i] * state_pred.angvel();
    }
}

void StateEstimator::correct(Eigen::Map<Vector3f> accel, Eigen::Map<Vector3f> gyro) {
    sensordata.segment<3>(YI_ACCEL) = accel;
    sensordata.segment<3>(YI_GYRO) = gyro;

    Eigen::Matrix<float, M, 1> innovation = sensordata - sens_pred; //TODO experiment w allocation
    Eigen::Matrix<float, M, M> innovation_cov = sens_jac * cov_pred * sens_jac.transpose() + sens_cov;
    Eigen::Matrix<float, N, M> filter_gain = cov_pred * sens_jac.transpose() * innovation_cov.inverse();
    Eigen::Matrix<float, N, N> cov_scale = Eigen::Matrix<float, N, N>::Identity() - filter_gain * sens_jac;
    state.vect = state_pred.vect + filter_gain * innovation;
    state.calculate();
    cov = cov_scale * cov_pred * cov_scale.transpose() + filter_gain * sens_cov * filter_gain.transpose();
}

std::ostream &operator<<(std::ostream &output, const StateEstimator &EKF) {
    output<<EKF.state.pos().transpose() << "," << EKF.state.euler().transpose() << ","
        <<EKF.state.vel().transpose() << "," << EKF.state.angvel().transpose() << ","
        <<EKF.state_pred.pos().transpose() << "," << EKF.state_pred.euler().transpose() << ","
        <<EKF.state_pred.vel().transpose() << "," << EKF.state_pred.angvel().transpose() << ","
        <<EKF.sensor_prediction.transpose();
    return output;
}

std::string estimator_CSV_header(void) {
    return "pos[3],euler[3],vel[3],angvel[3],pos_pred[3],euler_pred[3],vel_pred[3],angvel_pred[3],sensor_prediction[6]";
}
