/* state estimation
March 2022
TODO:
    legs
    move prediction to dynamics
    set_motion_noise
    alternate state estimator that purely believes sensors
*/
#ifndef STATEESTIMATION_HPP
#define STATEESTIMATION_HPP
#include "robot_state.hpp"
#include "sensorBoss.hpp"

class StateEstimator {
    MatrixXf cov, cov_pred; //state covariance
    MatrixXf motion_noise; //motion noise covariance
    MatrixXf sens_cov; //sensor covariance
    MatrixXf sens_jac; //d(sensor)/d(state)
    MatrixXf motion_jac; //d(new state)/d(state)
    float velocity_damping_per_tick{};
    void apply_damping();
public:
    StateEstimator(float timestep, float pos_stddev, float axis_stddev, float vel_stddev, float angvel_stddev);
    RobotState state, state_pred;
    const MatrixXf& state_covariance;
    const float dt;
    void predict();
    void correct(const SensorBoss &sensors);
    void set_damping_deceleration(float deceleration); // constant deceleration > 0 [m/s^2]
 };

#endif
