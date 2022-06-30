/* simplewalker state for state estimation 
March 2022 blame Niraj
?*/
#include "robot_state.hpp"
#include "logger.hpp"
#include <cmath>

RobotState::RobotState()
: vect(VectorXf::Zero(N)), R(R_), RT(RT_) {
    Vector3f unit_axis_init {1, 0, 0};
    float angle_init {M_PI_2};
    axis() = unit_axis_init * angle_init;
    R_cached_axis = axis();
    R_ = Eigen::AngleAxisf(angle_init, unit_axis_init).toRotationMatrix();
    RT_ = R_.transpose();
}

void RobotState::calculate(void) {
    /* if axis has changed enough, update rotation matrix.*/
    Vector3f delta_axis = axis() - R_cached_axis;
    float delta_angle = delta_axis.norm();
    if (delta_angle > rot_angle_update_thresh) {
        updateRotationMatrix_(delta_angle, delta_axis / delta_angle);
        stabilizeRotation_();
    } else ++cached_rotation_count;
}

void RobotState::updateRotationMatrix_(float delta_angle, const Vector3f &axis_normalized) {
    cached_rotation_count = 0;
    R_ = Eigen::AngleAxisf(delta_angle, axis_normalized) * R;
    RT_ = R.transpose();
    R_cached_axis = axis();
}

void RobotState::stabilizeRotation_(void) {
    Eigen::AngleAxisf angle_axis{R};
    axis() = angle_axis.axis() * angle_axis.angle();
    R_cached_axis = axis();
}


std::string state_CSV_header(void){
    return "pos[3],axis[3],vel[3],angvel[3]";
}
std::ostream &operator<<( std::ostream &output, const RobotState &State ) {
    /*output <<"Pos [m]: "<<State.pos().transpose()
            <<"\nEul [rad]: "<<State.axis().transpose()
            <<"\nVel [m/s]: "<<State.vel().transpose()
            <<"\nAngvel [rad/s]: "<<State.angvel().transpose()<<"\n"; */
    output <<State.pos().transpose()
            <<"  |  "<<State.axis().transpose()
            <<"  |  "<<State.vel().transpose()
            <<"  |  "<<State.angvel().transpose();
    return output;
}

Block3f RobotState::pos() {return vect.segment<3>(IDX_POS);}
const Vector3f RobotState::pos() const {return vect.segment<3>(IDX_POS);}
Block3f RobotState::axis() {return vect.segment<3>(IDX_AXIS);}
const Vector3f RobotState::axis() const {return vect.segment<3>(IDX_AXIS);}
Block3f RobotState::vel() {return vect.segment<3>(IDX_VEL);}
const Vector3f RobotState::vel() const {return vect.segment<3>(IDX_VEL);}
Block3f RobotState::angvel() {return vect.segment<3>(IDX_ANGVEL);}
const Vector3f RobotState::angvel() const {return vect.segment<3>(IDX_ANGVEL);}

