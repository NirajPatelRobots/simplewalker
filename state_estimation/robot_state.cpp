/* simplewalker state for state estimation 
March 2022 blame Niraj
TODO: flag to determine if calculated*/
#include "robot_state.hpp"


RobotState::RobotState() 
: R(R_), RT(RT_) {
    vect = VectorXf::Zero(N);
    R_ = Matrix3f::Identity();
    RT_ = Matrix3f::Identity();
}

void RobotState::calculate(void) {
    Eigen::Array3f s = euler().array().sin();
    Eigen::Array3f c = euler().array().cos();
    R_(0,0) = c[0]*c[1]*c[2] - s[0]*s[2];
    R_(0,1) = -c[2]*s[0] - c[0]*c[1]*s[2];
    R_(0,2) = c[0]*s[1];
    R_(1,0) = c[0]*s[2] + c[1]*c[2]*s[0];
    R_(1,1) = c[0]*c[2] - c[1]*s[0]*s[2];
    R_(1,2) = s[0]*s[1];
    R_(2,0) = -c[2]*s[1];
    R_(2,1) = s[1]*s[2];
    R_(2,2) = c[1];
    RT_ = R.transpose();
}

std::ostream &operator<<( std::ostream &output, const RobotState &State ) {
    output <<"Pos [m]: "<<State.pos().transpose()
            <<"\nEul [rad]: "<<State.euler().transpose()
            <<"\nVel [m/s]: "<<State.vel().transpose()
            <<"\nAngvel [rad/s]: "<<State.angvel().transpose()<<"\n";
    return output;
}

Block3f RobotState::pos() {return vect.segment<3>(STATEIDX_POS);}
const Vector3f RobotState::pos() const {return vect.segment<3>(STATEIDX_POS);}
Block3f RobotState::euler() {return vect.segment<3>(STATEIDX_EUL);}
const Vector3f RobotState::euler() const {return vect.segment<3>(STATEIDX_EUL);}
Block3f RobotState::vel() {return vect.segment<3>(STATEIDX_VEL);}
const Vector3f RobotState::vel() const {return vect.segment<3>(STATEIDX_VEL);}
Block3f RobotState::angvel() {return vect.segment<3>(STATEIDX_ANGVEL);}
const Vector3f RobotState::angvel() const {return vect.segment<3>(STATEIDX_ANGVEL);}
float& RobotState::alpha() {return vect(STATEIDX_EUL);}
float& RobotState::beta() {return vect(STATEIDX_EUL + 1);}
float& RobotState::gamma() {return vect(STATEIDX_EUL + 2);}

