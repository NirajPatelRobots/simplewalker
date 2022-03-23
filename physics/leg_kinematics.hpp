
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Matrix3f, Eigen::Vector3f;

const int SHIN_LENGTH = 0.1, THIGH_LENGTH = 0.1; // [m] (if you make them different, fix _fk_Jac_physTrig to do that)

// array indexes
const int RIGHT_IDX = 0, FORWARD_IDX = 1, UP_IDX = 2;
const Vector3f RIGHT_DIR = Vector3f(1,0,0);
const Vector3f FORWARD_DIR = Vector3f(0,1,0);
const Vector3f UP_DIR = Vector3f(0,0,1);
const Vector3f HORIZONTAL_DIR = RIGHT_DIR + FORWARD_DIR;
const Vector3f LEFTSCALE = Vector3f(-1., 1., 1.); // multiply vectors by LEFTSCALE to change right to left

void forward_kinematics(Vector3f &p_leg, const vector3f &theta);
void forward_kinematics(Vector3f &p_leg, Matrix3f &forward_Jacobian, const vector3f &theta);
void forward_Jacobian(Matrix3f &forward_Jacobian, const vector3f &theta);
void inverse_Jacobian(Matrix3f &inverse_Jacobian, const Matrix3f &forward_Jacobian, const vector3f &theta); //TODO
void forward_Jacobian_dot(Matrix3f &forward_Jacobian_dot, const vector3f &theta, const vector3f & omega); //TODO? currently numerical by dynamics
void inverse_kinematics(vector3f &theta, const vector3f &p_leg, bool ignore_failure); //TODO find theta from estimate and p_leg using solver

