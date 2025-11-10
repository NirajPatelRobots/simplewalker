/* simplewalker state for state estimation 
TODO:
    copy/move assignment operator
    make StatisticState with RobotState and covariance extracted from EKF?
    get state element std_dev from covariance
    load foot parameters from config
    separate state_t simple struct?
    acceleration change to acceleration() ?
March 2022 blame Niraj*/
#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP
#include "leg_kinematics.hpp"
#include <optional>
using Eigen::MatrixXf, Eigen::VectorXf;
typedef Eigen::VectorBlock<Eigen::VectorXf, 3> Block3f;

const int N = 12;
// DEFAULT_ROTATION != I to improve calculations in axis/R conversion and Jac_rotated_wrt_axis_angle
const Matrix3f DEFAULT_ROTATION {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};
const int FOOT_SHAPE_POINTS = 2;
typedef std::array<Vector3f, FOOT_SHAPE_POINTS> FootShape;

class RobotState;

class DynamicPose {
    /* Instantaneous State Information for a Robot
    stores robot state and calculate()s higher level info.
    R is rotation matrix, v_world_frame = R * v_body_frame.
    RT() is transposed rotation matrix, v_body_frame = RT * v_world_frame
    R is not always updated by calculate.
        rot_angle_update_thresh is maximum angle of rotation before an update to R.
    timestamp is the time this state represents. Position and velocity are instantaneous state variables at this time.
    acceleration is not part of the state vector. It's approximated constant over the timestep preceding this state measurement.
    */
    Matrix3f R_;
    Vector3f R_cached_axis;  // the axis vector representation of R.
    void increment_R_(float delta_angle, const Vector3f &axis_normalized);
    void set_R_(const Eigen::Ref<const Matrix3f> &new_R); // does not check error
    void set_R_(float angle, const Vector3f &axis_normalized);
    void setAxisFromR_();
public:
    enum index {IDX_POS = 0, IDX_AXIS = 3, IDX_VEL = 6, IDX_ANGVEL = 9};
    VectorXf vect;
    DynamicPose();
    void calculate();
    void set_R(const Eigen::Ref<const Matrix3f> &new_R);  // throws runtime_error if R_new not orthogonal
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
    inline Eigen::Transpose<const Matrix3f>RT() const {return R.transpose();}
};


struct FootInfo {
    FootShape points;
    Vector3f attach_point; // point the leg attaches to the body in body coordinates
    float radius;
    std::optional<float> spring_torque_per_rad;
    std::array<bool, FOOT_SHAPE_POINTS> points_are_sprung{};  // TODO
    FootInfo(FootShape polygon_, Vector3f attach_point, float radius_);
    FootInfo(FootShape polygon_, Vector3f attach_point, float radius_, float spring_k);
};

struct FootState { // TODO: Rename? velocity? remove R?
    int32_t timestamp_us{};
    bool in_contact() const;  // is this foot in solid contact with something
    Vector3f position;  // world frame
    Matrix3f R; // v_world_frame = R * v_foot_frame
    FootShape points{};  // world frame
    std::array<bool, FOOT_SHAPE_POINTS> points_contact{};  // is each point in solid contact with something
    std::optional<float> spring_angle{};
    FootState();
    void set(const Eigen::Ref<const Vector3f> &body_pos, const Eigen::Ref<const Vector3f> &leg_angles,
             const Matrix3f &R_body, const FootInfo &info, int32_t timestamp_us);
};


class RobotState : public DynamicPose {
public:
    RobotState();
    std::array<FootState, 2> foot_states;
};

#endif
