/* The sensor boss deals with the sensors. 
Responsible for:
    wrapping raw sensor data from the Communicator,
    sensor covariance, the accuracy of the sensors
    sensor prediction and sensor jacobian wrt state.
TODO:
    read data continuously incrementing (file) buffer instead of passively watching pointer 
    sensor bias/health estimate
    active covariance report*/
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
    void predict(const RobotState &state_pred, const RobotState &last_state, float dt, const Matrix3f *Jac_RT_Eul);

    //get elements
    const Vector3f accel() const;
    const Vector3f gyro() const;
};

