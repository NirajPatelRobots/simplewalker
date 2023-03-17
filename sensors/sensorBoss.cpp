#include "sensorBoss.hpp"

const size_t SensorBoss::IDX_ACCEL = 0; //TODO USE offsetof()
const size_t SensorBoss::IDX_GYRO = 3;

SensorBoss::SensorBoss(float accel_stddev, float gyro_stddev) :
 data_({}), data_pred_({}),
 data_vect_(vect_start(&data_)), vect_pred_(vect_start(&data_pred_)), bias(SensorVector::Zero()),
 cov(MatrixXf::Identity(M, M)), jac(MatrixXf::Zero(M, N)),
 data(data_), data_pred(data_pred_),
 data_vect(data_vect_), prediction(vect_pred_), covariance(cov), jacobian(jac) {
    cov.block<3,3>(IDX_ACCEL, IDX_ACCEL) *= accel_stddev * accel_stddev;
    cov.block<3,3>(IDX_GYRO, IDX_GYRO) *= gyro_stddev * gyro_stddev;
}

void SensorBoss::update_sensors(const SensorData *raw_data) {
    data_.timestamp_us = raw_data->timestamp_us;
    data_vect_ = Eigen::Map<const SensorVector>(vect_start(raw_data)) - bias;
}

void SensorBoss::predict(const RobotState &state_pred, float dt) {
    Vector3f imu_accel_pred_world = IMU_ORIENTATION * (state_pred.acceleration + GRAVITY_ACCEL);
    vect_pred_.segment<3>(IDX_ACCEL) = state_pred.RT * imu_accel_pred_world;
    // accel is not in state vector, so accelerometer is a fcn of velocity in state equations
    jac.block<3,3>(IDX_ACCEL, RobotState::IDX_VEL) = state_pred.RT * IMU_ORIENTATION / dt;
    Jac_rotated_wrt_axis_angle(jac.block<3,3>(IDX_ACCEL, RobotState::IDX_AXIS), -state_pred.axis(),
                               state_pred.RT, imu_accel_pred_world);

    vect_pred_.segment<3>(IDX_GYRO) = state_pred.RT * IMU_ORIENTATION * state_pred.angvel();
    jac.block<3,3>(IDX_GYRO, RobotState::IDX_ANGVEL) = state_pred.RT * IMU_ORIENTATION;
    Jac_rotated_wrt_axis_angle(jac.block<3,3>(IDX_GYRO, RobotState::IDX_AXIS), -state_pred.axis(),
                               state_pred.RT, IMU_ORIENTATION * state_pred.angvel());

    data_pred_.timestamp_us = state_pred.timestamp_us;
}

bool SensorBoss::data_is_valid(void) const {
    return true;    //TODO
}

void SensorBoss::set_bias(std::vector<float> accel_bias, std::vector<float> gyro_bias) {
    if (accel_bias.size() == 3)
        bias.segment<3>(IDX_ACCEL) = Eigen::Map<Vector3f>(accel_bias.data());
    else std::cerr << "Could not set sensor accel bias" << endl;
    if (gyro_bias.size() == 3)
        bias.segment<3>(IDX_GYRO) = Eigen::Map<Vector3f>(gyro_bias.data());
    else std::cerr << "Could not set sensor gyro bias" << endl;
}

bool SensorBoss::set_IMU_orientation(const std::vector<float> &orientation) {
    if (orientation.size() == 9) {
        IMU_ORIENTATION = Eigen::Map<const Matrix3f>(orientation.data());
        return true;
    }
    std::cerr << "Could not set IMU Rotation matrix from vector with size " << orientation.size() << endl;
    return false;
}

const Vector3f SensorBoss::accel() const {return data_vect.segment<3>(IDX_ACCEL);}
const Vector3f SensorBoss::gyro() const {return data_vect.segment<3>(IDX_GYRO);}

const Vector3f SensorBoss::accel_pred() const {return prediction.segment<3>(IDX_ACCEL);}
const Vector3f SensorBoss::gyro_pred() const {return prediction.segment<3>(IDX_GYRO);}


float *vect_start(SensorData *data) {return (float *)data->accel;}
const float *vect_start(const SensorData *data) {return (const float *)data->accel;}
