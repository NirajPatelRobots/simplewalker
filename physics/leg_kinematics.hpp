/* Library for kinematic calculations for walking robot
TODO:
    fk_Jacobian_dot? complicated, so numerically calculated by dynamics instead.
    Should fk_Jacobian or ik_Jacobian encourages backwards legs?
    inverse_kinematics
    test inverse jacobian
Adapted Mar 2022 from kinematics.py
By Niraj */
#include "walkerUtils.hpp"

const float SHIN_LENGTH = 0.1, THIGH_LENGTH = 0.1; // [m] (if you make them different, fix _fk_Jac_physTrig to do that)


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
