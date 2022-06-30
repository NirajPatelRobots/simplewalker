/* Library for kinematic calculations for walking robot
TODO:
    fk_Jacobian_dot? complicated, so numerically calculated by dynamics instead.
    Should fk_Jacobian or ik_Jacobian encourages backwards legs?
    inverse_kinematics
    move non-leg stuff somewhere else
    BUG: Jac_rotated_wrt_axis_angle

Adapted Mar 2022 from kinematics.py
By Niraj */
#include <cmath>
#include <assert.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Matrix3f, Eigen::Vector3f;
#ifndef M_PI  // I can't believe I have to do this
#define M_PI (3.14159265358979323846)
#define M_PI_2 (1.57079632679489661923)
#endif

const float SHIN_LENGTH = 0.1, THIGH_LENGTH = 0.1; // [m] (if you make them different, fix _fk_Jac_physTrig to do that)

// array indexes
const int RIGHT_IDX = 0, FORWARD_IDX = 1, UP_IDX = 2;
const Vector3f RIGHT_DIR = Vector3f(1,0,0);
const Vector3f FORWARD_DIR = Vector3f(0,1,0);
const Vector3f UP_DIR = Vector3f(0,0,1);
const Vector3f HORIZONTAL_DIR = RIGHT_DIR + FORWARD_DIR;
const Vector3f LEFTSCALE = Vector3f(-1., 1., 1.); // multiply vectors by LEFTSCALE to change right to left


/*given motor angles, find the expected position of the foot.
theta(0) is the hip out angle, zero when the leg is pointing down, positive outwards from the side of the body.
theta(1) is the hip swing angle, zero when the leg is pointing down, positive forwards
theta(2) is the knee angle, zero when the knee is straight
return_Jacobian: True to return (p_leg, fk_Jacobian), False (default) returns p_leg.
    See versions with or without Jacobian
sets relative leg position p_leg 3-vector in robot coordinates*/
void forward_kinematics(Vector3f &p_leg, const Vector3f &theta);
void forward_kinematics(Vector3f &p_leg, Matrix3f &forward_Jacobian, const Vector3f &theta);

/*given motor angles, find the Jacobian d(p_leg)/d(motor_angle)
    where p_leg is relative leg position array of [forward, out, up].
theta(0) is the hip outward motor angle, zero when the leg is pointing down, positive outwards from the side of the body.
theta(1) is the hip forward motor angle, zero when the leg is pointing down, positive forwards
theta(2) is the knee angle, zero when the knee is straight, positive forwards
down is towards the ground, forward and out are body orientation, all 3 are perpendicular to each other
using both forward_Jacobian() and forward_kinematics() is slower than forward_kinematics() with jacobian
sets Jacobian, a 3x3 matrix*/
void forward_Jacobian(Matrix3f &forward_Jacobian, const Vector3f &theta);

/*find the inverse of the fk Jacobian. Deals with singularity.
Near theta2=0, Jacobian gets an added term that moves leg forward
fk_Jac is the forward jacobian, angle is the associated leg angles*/
void inverse_Jacobian(Matrix3f &inverse_Jacobian, const Matrix3f &forward_Jacobian, const Vector3f &theta);

/* given leg angles and angular velocity, find the time derivative of the Jacobian returned by fk_Jacobian.
theta(0) is the hip outward motor angle, zero when the leg is pointing down, positive outwards from the side of the body.
theta(1) is the hip forward motor angle, zero when the leg is pointing down, positive forwards
theta(2) is the knee angle, zero when the knee is straight, positive forwards

returns 3x3 time derivative of Jacobian matrix"""
#TODO? complicated, so numerically calculated by motionController instead. */
void forward_Jacobian_dot(Matrix3f &forward_Jacobian_dot, const Vector3f &theta, const Vector3f & omega); //TODO? currently numerical by dynamics

void inverse_kinematics(Vector3f &theta, const Vector3f &p_leg, bool ignore_failure); //TODO find theta from estimate and p_leg using solver




/*------------------------------------------------------------------------*/
// rotation kinematics 


/* find the Jacobian of a rotated vector with respect to the axis-angle vector.
axis_angle is the vector whose direction is the axis of rotation.
    the robot_state axis_angle is from body to world frame. That corresponds to unrotated_vector in body coordinates.
    if you want the jacobian of a vector in body frame, then axis_angle should be the - of the body to world axis_angle in robot_state.
    the magnitude of axis_angle is the angle of rotation.
unrotated_vector is the vector before rotation. 
sets 3x3 Jacobian  */
void Jac_rotated_wrt_axis_angle(Eigen::Ref<Matrix3f> Jacobian, const Vector3f &axis_angle, const Vector3f &unrotated_vector);


/* Given a vector, calculate its unit vector and the jacobian of the unit vector wrt the vector
*/
void jacobian_unitvec_wrt_vec(Matrix3f &jacobian, const Vector3f &vec);
