/* simplewalker state for state estimation 
March 2022 blame Niraj
?*/
#include "robot_state.hpp"
#include "logger.hpp"

RobotState::RobotState()
: R_(DEFAULT_ROTATION), vect(VectorXf::Zero(N)), acceleration(Vector3f::Zero()), timestamp_us(), R(R_), RT(RT_) {
    set_R_(R);
    setAxisFromR_();
}

/* if axis has changed enough, update rotation matrix.*/
void RobotState::calculate(void) {
    float delta_angle = (axis() - R_cached_axis).norm();
    float axis_angle = axis().norm();
    if (axis_angle > 2 * M_PI) {
        axis() -= 2 * M_PI * axis() / axis_angle;
        axis_angle -= 2 * M_PI;
    }
    if (delta_angle > rot_angle_update_thresh) {
        cached_rotation_count = 0;
        set_R_(axis_angle, axis() / axis_angle);
    } else ++cached_rotation_count;
}

void RobotState::set_R(const Eigen::Ref<const Matrix3f> &new_R) {
    if ((new_R * new_R.transpose() - Matrix3f::Identity()).norm() > 1e-5 ) {
        std::cerr << "Err: attempted to set non-orthogonal R\n";
        throw std::runtime_error("ERR: attempted to set non-orthogonal R");
    }
    set_R_(new_R);
}

void RobotState::increment_R_(float delta_angle, const Vector3f &axis_normalized) {
    set_R_(Eigen::AngleAxisf(delta_angle, axis_normalized) * R);
}

void RobotState::set_R_(float angle, const Vector3f &axis_normalized) {
    set_R_(Eigen::AngleAxisf(angle, axis_normalized).toRotationMatrix());
}

void RobotState::set_R_(const Eigen::Ref<const Matrix3f> &new_R) {
    R_ = new_R;
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

