/* state estimation without legs
    March 2022
*/
#include "state_estimation.hpp"
#include <cmath>

#define YI_ACCEL 0
#define YI_GYRO 3
const int M = 6;
 

StateEstimator::StateEstimator(float accel_stddev, float gyro_stddev, float timestep,
                                float pos_stddev, float euler_stddev, float vel_stddev, float angvel_stddev)
    : sensor_prediction(sens_pred), state_covariance(cov), dt(timestep) {
    sensordata = VectorXf::Zero(M);
    cov = 0.01 * MatrixXf::Identity(N, N);
    sens_pred = VectorXf::Zero(M);
    sens_cov = MatrixXf::Identity(M, M) * dt;
    sens_cov.block<3,3>(YI_ACCEL, YI_ACCEL) *= pow(accel_stddev, 2);
    sens_cov.block<3,3>(YI_GYRO, YI_GYRO) *= pow(gyro_stddev, 2);
    motion_noise = MatrixXf::Identity(N, N) * dt;
    motion_noise.block<3,3>(STATEIDX_POS, STATEIDX_POS) *= pow(pos_stddev, 2);
    motion_noise.block<3,3>(STATEIDX_EUL, STATEIDX_EUL) *= pow(euler_stddev, 2);
    motion_noise.block<3,3>(STATEIDX_VEL, STATEIDX_VEL) *= pow(vel_stddev, 2);
    motion_noise.block<3,3>(STATEIDX_ANGVEL, STATEIDX_ANGVEL) *= pow(angvel_stddev, 2);
    // set sens_jac and motion_jac from models
    motion_jac = MatrixXf::Identity(N, N);
    motion_jac.block<3,3>(STATEIDX_POS, STATEIDX_VEL) = Matrix3f::Identity() * dt;
    motion_jac.block<3,3>(STATEIDX_EUL, STATEIDX_ANGVEL) = Matrix3f::Identity() * dt;
    sens_jac = MatrixXf::Zero(M, N);
}

void StateEstimator::predict() {
    //set state_pred from state (motion model)
    state_pred.pos() = state.pos() + state.vel() * dt;
    state_pred.euler() = state.euler() + state.angvel() * dt;
    state_pred.vel() = state.vel();
    state_pred.angvel() = state.angvel();
    state_pred.calculate();
    cov_pred = motion_jac * cov * motion_jac.transpose() + motion_noise;
    //set sens_pred and sens_jac from state_pred (sensor model)
    sens_pred.segment<3>(YI_ACCEL) = state_pred.RT * (state_pred.vel() - state.vel()) / dt;
    sens_jac.block<3,3>(YI_ACCEL, STATEIDX_VEL) = state_pred.RT / dt;
    sens_pred.segment<3>(YI_GYRO) = /*state_pred.RT * */state_pred.angvel() / dt;
    sens_jac.block<3,3>(YI_GYRO, STATEIDX_ANGVEL) = Matrix3f::Identity() / dt; //state_pred.RT / dt;;
}

void StateEstimator::correct(Vector3f accel, Vector3f gyro) {
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
        <<EKF.sensor_prediction.transpose() << "\n";
    return output;
}

std::string estimator_CSV_header(void) {
    return "time,pos[3],euler[3],vel[3],angvel[3],pos_pred[3],euler_pred[3],vel_pred[3],angvel_pred[3],"
            "sensor_prediction[6]\n";
}
