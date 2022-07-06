/* state estimation
March 2022
TODO:
    legs
    velocity damping
*/
#include "robot_state.hpp"
#include "sensorBoss.hpp"

class StateEstimator {
    MatrixXf cov, cov_pred; //state covariance
    MatrixXf motion_noise; //motion noise covariance
    MatrixXf sens_cov; //sensor covariance
    MatrixXf sens_jac; //d(sensor)/d(state)
    MatrixXf motion_jac; //d(new state)/d(state)
    SensorBoss &sensors;
public:
    StateEstimator(float timestep, SensorBoss &sensorBoss,
                   float pos_stddev, float axis_stddev, float vel_stddev, float angvel_stddev);
    RobotState state, state_pred;
    const MatrixXf& state_covariance;
    const float dt;
    void predict(void);
    void correct(void);
};
