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
*/
#ifndef SENSORBOSS_HPP
#define SENSORBOSS_HPP
#include "robot_state.hpp"
const int M = 6;
typedef Eigen::Vector<float, M> SensorVector;

class SensorBoss {
protected:
    SensorData data_, data_pred_;
    Eigen::Map<SensorVector> data_vect_, vect_pred_;
    SensorVector bias;
    MatrixXf cov;
    MatrixXf jac;
    //Eigen::DiagonalMatrix<float, M> cov//TODO UNDERSTAND DIAGONAL
public:
    const static size_t IDX_ACCEL, IDX_GYRO;
    const SensorData &data, &data_pred;
    const Eigen::Map<SensorVector> &data_vect, &prediction;
    const MatrixXf &covariance;
    const MatrixXf &jacobian;

    SensorBoss(float accel_stddev, float gyro_stddev);
    void update_sensors(const SensorData *raw_data);
    void set_bias(std::vector<float> accel_bias, std::vector<float> gyro_bias);
    // predict() sets prediction and jacobian from state_pred (sensor model)
    void predict(const RobotState &state_pred, const RobotState &last_state, float dt);
    bool data_is_valid(void) const;
    //get elements
    const Vector3f accel() const;
    const Vector3f gyro() const;
    const Vector3f accel_pred() const;
    const Vector3f gyro_pred() const;
};

float *vect_start(SensorData *data);
const float *vect_start(const SensorData *data);

const Vector3f IMU_GRAVITY(0, 9.81, 0); // gravity on IMU in world frame
#endif
