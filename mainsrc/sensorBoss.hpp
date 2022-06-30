/* The sensor boss deals with the sensors. 
Responsible for:
    wrapping raw sensor data from the Communicator,
    sensor covariance, the inverse precision of the sensors
    sensor prediction and sensor jacobian wrt state.
TODO:
    try covariance asDiagonal
    sensor bias/health estimate
    active covariance report
*/
#ifndef SENSORBOSS_HPP
#define SENSORBOSS_HPP
#include "robot_state.hpp"
const int M = 6;

class SensorBoss {
protected:
    Eigen::Vector<float, M> sens_pred;
    //Eigen::DiagonalMatrix<float, M> //TODO UNDERSTAND DIAGONAL
    MatrixXf cov;
    MatrixXf jac;
public:
    const static size_t IDX_ACCEL, IDX_GYRO;
    Eigen::Map<const VectorXf> data;
    const Eigen::Vector<float, M> &prediction;
    const MatrixXf &covariance;
    const MatrixXf &jacobian;

    SensorBoss(const float *msgsensordata, float accel_stddev, float gyro_stddev);
    void predict(const RobotState &state_pred, const RobotState &last_state, float dt);
    //if initialized from a data stream instead of an updating vector, incrementdata increments to the next data
    void incrementdata(int numtoskip); //numtoskip is how many extra floats to skip forward when incrementing the buffer
    void calibrate(); // use this data to estimate calibration

    //get elements
    const Vector3f accel() const;
    const Vector3f gyro() const;
};
#endif
