/* state estimation
March 2022
TODO:
    legs
*/
#include "state_estimation.hpp"


StateEstimator::StateEstimator(float timestep, SensorBoss &sensorBoss,
                                float pos_stddev, float euler_stddev, float vel_stddev, float angvel_stddev)
    : sensors(sensorBoss), state_covariance(cov), dt(timestep) {
    cov = 0.01 * MatrixXf::Identity(N, N);
    for (int i = 0; i < 3; i++) {
        Jac_RT_Eul[i] = Matrix3f::Zero();
    }
    motion_noise = MatrixXf::Identity(N, N) * dt;
    motion_noise.block<3,3>(RobotState::IDX_POS, RobotState::IDX_POS) *= pow(pos_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_EUL, RobotState::IDX_EUL) *= pow(euler_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_VEL, RobotState::IDX_VEL) *= pow(vel_stddev, 2);
    motion_noise.block<3,3>(RobotState::IDX_ANGVEL, RobotState::IDX_ANGVEL) *= pow(angvel_stddev, 2);
    // set motion_jac from motion model
    motion_jac = MatrixXf::Identity(N, N);
    motion_jac.block<3,3>(RobotState::IDX_POS, RobotState::IDX_VEL) = Matrix3f::Identity() * dt;
    motion_jac.block<3,3>(RobotState::IDX_EUL, RobotState::IDX_ANGVEL) = Matrix3f::Identity() * dt;
}

void StateEstimator::predict() {
    //set state_pred from state (motion model)
    state_pred.pos() = state.pos() + state.vel() * dt;
    state_pred.euler() = state.euler() + state.angvel() * dt;
    state_pred.vel() = state.vel();
    state_pred.angvel() = state.angvel();
    state_pred.calculate(Jac_RT_Eul);
    cov_pred = motion_jac * cov * motion_jac.transpose() + motion_noise;
    //set sens_pred and sens_jac from state_pred (sensor model)
    sensors.predict(state_pred, state, dt, Jac_RT_Eul);
}

void StateEstimator::correct() {
    Eigen::Matrix<float, M, 1> innovation = sensors.data - sensors.prediction; //TODO experiment w allocation
    Eigen::Matrix<float, M, M> innovation_cov = sensors.jacobian * cov_pred * sensors.jacobian.transpose() + sensors.covariance;
    Eigen::Matrix<float, N, M> filter_gain = cov_pred * sensors.jacobian.transpose() * innovation_cov.inverse();
    Eigen::Matrix<float, N, N> cov_scale = Eigen::Matrix<float, N, N>::Identity() - filter_gain * sensors.jacobian;
    state.vect = state_pred.vect + filter_gain * innovation;
    state.calculate();
    cov = cov_scale * cov_pred * cov_scale.transpose() + filter_gain * sensors.covariance * filter_gain.transpose();

    state.vel() *= (1 - 10*dt); // DEBUG DEBUG TODO REMOVE DAMPING
}

std::ostream &operator<<(std::ostream &output, const StateEstimator &EKF) {
    output<<EKF.state.pos().transpose() << "," << EKF.state.euler().transpose() << ","
        <<EKF.state.vel().transpose() << "," << EKF.state.angvel().transpose() << ","
        <<EKF.state_pred.pos().transpose() << "," << EKF.state_pred.euler().transpose() << ","
        <<EKF.state_pred.vel().transpose() << "," << EKF.state_pred.angvel().transpose();
    return output;
}

std::string estimator_CSV_header(void) {
    return "pos[3],euler[3],vel[3],angvel[3],pos_pred[3],euler_pred[3],vel_pred[3],angvel_pred[3]";
}