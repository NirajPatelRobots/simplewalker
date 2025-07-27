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

void StateEstimator::predict() {
    //set state_pred from state (motion model)
    state_pred.pos() = state.pos() + (state.vel() + .5 * state_pred.acceleration * dt) * dt;
    state_pred.axis() = state.axis() + state.angvel() * dt;
    state_pred.vel() = state.vel() + state_pred.acceleration * dt;
    state_pred.angvel() = state.angvel();
    state_pred.calculate();
    cov_pred = motion_jac * cov * motion_jac.transpose() + motion_noise;
    state_pred.timestamp_us = state.timestamp_us + (uint32_t)(dt * 1e6);
}

void StateEstimator::correct(const SensorBoss &sensors) {
    SensorVector innovation = sensors.data_vect - sensors.prediction; // TODO experiment w allocation dynamic matrices
    Eigen::Matrix<float, M, M> innovation_cov = sensors.jacobian * cov_pred * sensors.jacobian.transpose() + sensors.covariance;
    Eigen::Matrix<float, N, M> filter_gain = cov_pred * sensors.jacobian.transpose() * innovation_cov.inverse();
    Eigen::Matrix<float, N, N> cov_scale = Eigen::Matrix<float, N, N>::Identity() - filter_gain * sensors.jacobian;
    Vector3f prev_velocity{state.vel()};
    state.vect = state_pred.vect + filter_gain * innovation;
    apply_damping();
    state.acceleration = (state.vel() - prev_velocity) / dt;
    state.calculate();
    cov = cov_scale * cov_pred * cov_scale.transpose() + filter_gain * sensors.covariance * filter_gain.transpose();
    state.timestamp_us = sensors.data.timestamp_us;
}

void StateEstimator::set_damping_deceleration(float deceleration) {
    velocity_damping_per_tick = deceleration * dt;
}

void StateEstimator::apply_damping() {
    const float velocity_magnitude = state.vel().norm();
    if (velocity_magnitude > velocity_damping_per_tick)
        state.vel() *= (velocity_magnitude - velocity_damping_per_tick) / velocity_magnitude;
    else
        state.vel() = Vector3f::Zero();
}
