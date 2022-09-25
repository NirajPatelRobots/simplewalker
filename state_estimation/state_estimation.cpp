#include "state_estimation.hpp"


StateEstimator::StateEstimator(float timestep,
                                float pos_stddev, float axis_stddev, float vel_stddev, float angvel_stddev)
    : state(), state_pred(), state_covariance(cov), dt(timestep) {
    cov = 0.01 * MatrixXf::Identity(N, N);
    motion_noise = MatrixXf::Identity(N, N) * dt;
    motion_noise.block<3,3>(RobotState::IDX_POS, RobotState::IDX_POS) *= pow(pos_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_AXIS, RobotState::IDX_AXIS) *= pow(axis_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_VEL, RobotState::IDX_VEL) *= pow(vel_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_ANGVEL, RobotState::IDX_ANGVEL) *= pow(angvel_stddev, 2);
    // set motion_jac from motion model
    motion_jac = MatrixXf::Identity(N, N);
    motion_jac.block<3,3>(RobotState::IDX_POS, RobotState::IDX_VEL) = Matrix3f::Identity() * dt;
    motion_jac.block<3,3>(RobotState::IDX_AXIS, RobotState::IDX_ANGVEL) = Matrix3f::Identity() * dt;
}

void StateEstimator::predict(void) {
    //set state_pred from state (motion model)
    state_pred.pos() = state.pos() + state.vel() * dt;
    state_pred.axis() = state.axis() + state.angvel() * dt;
    state_pred.vel() = state.vel();
    state_pred.angvel() = state.angvel();
    state_pred.calculate();
    cov_pred = motion_jac * cov * motion_jac.transpose() + motion_noise;
}

void StateEstimator::correct(const SensorBoss &sensors) {
    SensorVector innovation = sensors.data_vect - sensors.prediction; // TODO experiment w allocation dynamic matrices
    Eigen::Matrix<float, M, M> innovation_cov = sensors.jacobian * cov_pred * sensors.jacobian.transpose() + sensors.covariance;
    Eigen::Matrix<float, N, M> filter_gain = cov_pred * sensors.jacobian.transpose() * innovation_cov.inverse();
    Eigen::Matrix<float, N, N> cov_scale = Eigen::Matrix<float, N, N>::Identity() - filter_gain * sensors.jacobian;
    state.vect = state_pred.vect + filter_gain * innovation;
    state.calculate();
    cov = cov_scale * cov_pred * cov_scale.transpose() + filter_gain * sensors.covariance * filter_gain.transpose();
    state.vel() *= (1 - 10*dt); // DEBUG DEBUG TODO REMOVE DAMPING
}
