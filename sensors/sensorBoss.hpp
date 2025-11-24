/* The sensor boss deals with the sensors. 
Responsible for:
    wrapping raw sensor data from the Communicator,
    sensor covariance, the inverse precision of the sensors
    sensor prediction and sensor jacobian wrt state.
TODO:
    try covariance asDiagonal
    health estimate
    active covariance
    data_is_valid()
    SensorBoss template can contain multiple IMUs and leg sensors
    hide IDX_ACCEL and IDX_GYRO?
*/
#ifndef SENSORBOSS_HPP
#define SENSORBOSS_HPP
#include "robot_state.hpp"
#include "sensor_data.h"
const int M = 6;
typedef Eigen::Vector<float, M> SensorVector;

class SensorBoss {
protected:
    SensorData data_, data_pred_; // unbiased
    Eigen::Map<SensorVector> data_vect_, vect_pred_;
    SensorVector bias;
    MatrixXf cov;
    MatrixXf jac;
    Matrix3f IMU_ORIENTATION{Matrix3f::Identity()}; // imu_measurement = RT * IMU_ORIENTATION * measurement_in_world_frame
    //Eigen::DiagonalMatrix<float, M> cov//TODO UNDERSTAND DIAGONAL
public:
    const static size_t IDX_ACCEL, IDX_GYRO;
    const SensorData &data, &data_pred;
    const Eigen::Map<SensorVector> &data_vect, &prediction;
    const MatrixXf &covariance;
    const MatrixXf &jacobian;

    SensorBoss(float accel_stddev, float gyro_stddev);
    void update_sensors(const SensorData *raw_data);
    // predict() sets prediction and jacobian from state_pred (sensor model)
    void predict(const RobotState &state_pred, float dt);
    bool data_is_valid() const;
    void set_bias(std::vector<float> accel_bias, std::vector<float> gyro_bias);
    bool set_IMU_orientation(const std::vector<float> &orientation);
    //get elements
    const Vector3f accel() const;
    const Vector3f gyro() const;
    const Vector3f accel_pred() const;
    const Vector3f gyro_pred() const;
};

float *vect_start(SensorData *data);
const float *vect_start(const SensorData *data);

#endif
