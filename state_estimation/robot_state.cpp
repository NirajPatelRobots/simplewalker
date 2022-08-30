/* simplewalker state for state estimation 
March 2022 blame Niraj
?*/
#include "robot_state.hpp"
#include "logger.hpp"

RobotState::RobotState()
: R_(DEFAULT_ROTATION), vect(VectorXf::Zero(N)), R(R_), RT(RT_) {
    setAxisFromR_();
    updateRotationMatrix_(0, Vector3f(1, 0, 0));
}

void RobotState::calculate(void) {
    /* if axis has changed enough, update rotation matrix.*/
    Vector3f delta_axis = axis() - R_cached_axis;
    float delta_angle = delta_axis.norm();
    if (delta_angle > rot_angle_update_thresh) {
        cached_rotation_count = 0;
        updateRotationMatrix_(delta_angle, delta_axis / delta_angle);
        setAxisFromR_();
    } else ++cached_rotation_count;
}

void RobotState::updateRotationMatrix_(float delta_angle, const Vector3f &axis_normalized) {
    R_ = Eigen::AngleAxisf(delta_angle, axis_normalized) * R;
    RT_ = R.transpose();
    R_cached_axis = axis();
}

void RobotState::setAxisFromR_(void) {
    Eigen::AngleAxisf angle_axis{R};
    axis() = angle_axis.axis() * angle_axis.angle();
    R_cached_axis = axis();
}

Block3f RobotState::pos() {return vect.segment<3>(IDX_POS);}
const Vector3f RobotState::pos() const {return vect.segment<3>(IDX_POS);}
Block3f RobotState::axis() {return vect.segment<3>(IDX_AXIS);}
const Vector3f RobotState::axis() const {return vect.segment<3>(IDX_AXIS);}
Block3f RobotState::vel() {return vect.segment<3>(IDX_VEL);}
const Vector3f RobotState::vel() const {return vect.segment<3>(IDX_VEL);}
Block3f RobotState::angvel() {return vect.segment<3>(IDX_ANGVEL);}
const Vector3f RobotState::angvel() const {return vect.segment<3>(IDX_ANGVEL);}

