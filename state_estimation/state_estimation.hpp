#include "robot_state.hpp"

class StateEstimator {
    MatrixXf cov, cov_pred; //state covariance
    VectorXf sensordata, sens_pred;
    MatrixXf motion_noise; //motion noise covariance
    MatrixXf sens_cov; //sensor covariance
    MatrixXf sens_jac; //d(sensor)/d(state)
    MatrixXf motion_jac; //d(new state)/d(state)
    Matrix3f Jac_RT_Eul[3]; //d(RT matrix)/d(euler angle[3])
public:
    StateEstimator(float accel_stddev, float gyro_stddev, float timestep,
                   float pos_stddev, float euler_stddev, float vel_stddev, float angvel_stddev);
    RobotState state, state_pred;
    const VectorXf& sensor_prediction;
    const MatrixXf& state_covariance;
    const float dt;
    void predict(void);
    void correct(Eigen::Map<Vector3f> accel, Eigen::Map<Vector3f> gyro);
};

std::ostream &operator<<(std::ostream &output, const StateEstimator &Estimator);
std::string estimator_CSV_header(void);
