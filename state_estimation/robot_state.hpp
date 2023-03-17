/* simplewalker state for state estimation 
TODO:
    stabilizeRotation characterize drift
    copy assignment operator
    make covariance part of state instead of EKF
    get state element std_dev from covariance
    separate state_t simple struct?
March 2022 blame Niraj*/
#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP
#include "leg_kinematics.hpp"
using Eigen::MatrixXf, Eigen::VectorXf;
typedef Eigen::VectorBlock<Eigen::VectorXf, 3> Block3f;

const int N = 12;
const Matrix3f DEFAULT_ROTATION {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};

class RobotState {
    /* Instantaneous State Information for a Robot
    stores robot state and calculate()s higher level info.
    R is rotation matrix, v_world_frame = R * v_body_frame.
    RT is transposed rotation matrix, v_body_frame = RT * v_world_frame
    updating R with small rotations is numerically awkward, so R is not always updated by calculate.
        rot_angle_update_thresh is maximum angle of rotation before an update to R.
    timestamp is the time this state represents. Position and velocity are instantaneous state variables at this time.
    acceleration is not part of the state vector. It's approximated constant over the timestep preceding this state measurement.
    */
    Matrix3f R_, RT_;
    Vector3f R_cached_axis;  // the axis vector representation of R. 
    void updateRotationMatrix_(float delta_angle, const Vector3f &axis_normalized);
    void setAxisFromR_();
public:
    enum index {IDX_POS = 0, IDX_AXIS = 3, IDX_VEL = 6, IDX_ANGVEL = 9};
    VectorXf vect;
    RobotState();
    void calculate();
    float rot_angle_update_thresh = 0.0001;
    unsigned cached_rotation_count = 0;
    //state vector elements
    Block3f pos();
    const Vector3f pos() const;
    Block3f axis();
    const Vector3f axis() const;
    Block3f vel();
    const Vector3f vel() const;
    Block3f angvel();
    const Vector3f angvel() const;
    // additional state elements
    Vector3f acceleration; // average acceleration during the preceding timestep
    uint32_t timestamp_us;
    // calculated elements
    const Matrix3f& R;
    const Matrix3f& RT;
};

#endif
