/* simplewalker state for state estimation 
March 2022 blame Niraj
TODO: 
?*/
#include "robot_state.hpp"
#include <cmath>

RobotState::RobotState() 
: R(R_), RT(RT_) {
    vect = VectorXf::Zero(N);
    euler()[1] = M_PI / 2;
    calculate();
}

void RobotState::calculate(void) {
    /*All the trig for rotation matrices from ZYZ Euler angles */
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

void RobotState::calculate(Matrix3f *dRT_deul) {
    /*All the trig for rotation matrices from ZYZ Euler angles
    also does derivative of inverse rotation matrix w.r.t. each euler angle */
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
    dRT_deul[0](0,0) = -s[0]*c[1]*c[2] - c[0]*s[2];
    dRT_deul[0](0,1) = -s[0]*s[2] + c[0]*c[1]*c[2];
    //dRT_deul[0](0,2) = 0;
    dRT_deul[0](1,0) = -c[0]*c[2] + s[0]*c[1]*s[2];
    dRT_deul[0](1,1) = -s[0]*c[2] - c[1]*c[0]*s[2];
    //dRT_deul[0](1,2) = 0;
    dRT_deul[0](2,0) = -s[0]*s[1];
    dRT_deul[0](2,1) = -c[0]*s[1];
    //dRT_deul[0](2,2) = 0;
    
    dRT_deul[1](0,0) = -c[0]*s[1]*c[2];
    dRT_deul[1](0,1) = -s[0]*s[1]*c[2];
    dRT_deul[1](0,2) = -c[2]*c[1];
    dRT_deul[1](1,0) = c[0]*s[1]*s[2];
    dRT_deul[1](1,1) = s[1]*s[0]*s[2];
    dRT_deul[1](1,2) = c[1]*s[2];
    dRT_deul[1](2,0) = c[0]*c[1];
    dRT_deul[1](2,1) = s[0]*c[1];
    dRT_deul[1](2,2) = s[1];
    
    dRT_deul[2](0,0) = -c[0]*c[1]*s[2] - s[0]*c[2];
    dRT_deul[2](0,1) = c[0]*c[2] - c[1]*s[2]*s[0];
    dRT_deul[2](0,2) = s[2]*s[1];
    dRT_deul[2](1,0) = s[2]*s[0] - c[0]*c[1]*c[2];
    dRT_deul[2](1,1) = -c[0]*s[2] - c[1]*s[0]*c[2];
    dRT_deul[2](1,2) = s[1]*c[2];
    //dRT_deul[2](2,0) = 0;
    //dRT_deul[2](2,1) = 0;
    //dRT_deul[2](2,2) = 0;
}

std::string state_CSV_header(void){
    return "pos[3],euler[3],vel[3],angvel[3]";
}
std::ostream &operator<<( std::ostream &output, const RobotState &State ) {
    /*output <<"Pos [m]: "<<State.pos().transpose()
            <<"\nEul [rad]: "<<State.euler().transpose()
            <<"\nVel [m/s]: "<<State.vel().transpose()
            <<"\nAngvel [rad/s]: "<<State.angvel().transpose()<<"\n"; */
    output <<State.pos().transpose()
            <<"  |  "<<State.euler().transpose()
            <<"  |  "<<State.vel().transpose()
            <<"  |  "<<State.angvel().transpose();
    return output;
}

Block3f RobotState::pos() {return vect.segment<3>(IDX_POS);}
const Vector3f RobotState::pos() const {return vect.segment<3>(IDX_POS);}
Block3f RobotState::euler() {return vect.segment<3>(IDX_EUL);}
const Vector3f RobotState::euler() const {return vect.segment<3>(IDX_EUL);}
Block3f RobotState::vel() {return vect.segment<3>(IDX_VEL);}
const Vector3f RobotState::vel() const {return vect.segment<3>(IDX_VEL);}
Block3f RobotState::angvel() {return vect.segment<3>(IDX_ANGVEL);}
const Vector3f RobotState::angvel() const {return vect.segment<3>(IDX_ANGVEL);}
float &RobotState::alpha() {return vect(IDX_EUL);}
float &RobotState::beta() {return vect(IDX_EUL + 1);}
float &RobotState::gamma() {return vect(IDX_EUL + 2);}

