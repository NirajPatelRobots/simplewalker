/* Library for kinematic calculations for walking robot
TODO:
    fk_Jacobian_dot? complicated, so numerically calculated by dynamics instead.
    Should fk_Jacobian or ik_Jacobian encourages backwards legs?

Adapted Mar 2022 from kinematics.py
By Niraj
*/
#include "leg_kinematics.hpp"
#include <cmath>
void fk_Jac_physTrig(Matrix3f &Jac, const vector3f &sines, const vector3f &cosines);

void forward_kinematics(Vector3f &p_leg, const vector3f &theta) {
    /*given motor angles, find the expected position of the foot.
    theta(0) is the hip out angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta(1) is the hip swing angle, zero when the leg is pointing down, positive forwards
    theta(2) is the knee angle, zero when the knee is straight
    return_Jacobian: True to return (p_leg, fk_Jacobian), False (default) returns p_leg.
        See also version with Jacobian
    
    sets relative leg position p_leg 3-vector in robot coordinates*/
    Vector3f sines = theta.array().sin();
    Vector3f cosines = theta.array().cos();
    p_leg(FORWARD_IDX) = (THIGH_LENGTH * sines(1) + SHIN_LENGTH * sines(2)) * cosines(0);
    p_leg(RIGHT_IDX) = (THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * sines(0);
    p_leg(UP_IDX) = -(THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * cosines(0);
}

void forward_kinematics(Vector3f &p_leg, Matrix3f &forward_Jacobian, const vector3f &theta) {
    /*given motor angles, find the expected position of the foot.
    theta(0) is the hip out angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta(1) is the hip swing angle, zero when the leg is pointing down, positive forwards
    theta(2) is the knee angle, zero when the knee is straight
    
    sets relative leg position p_leg 3-array in robot coordinates
    and sets fk_Jac, the Jacobian d(p_leg)/d(motor_angle) */
    Vector3f sines = theta.array().sin();
    Vector3f cosines = theta.array().cos();
    p_leg(FORWARD_IDX) = (THIGH_LENGTH * sines(1) + SHIN_LENGTH * sines(2)) * cosines(0);
    p_leg(RIGHT_IDX) = (THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * sines(0);
    p_leg(UP_IDX) = -(THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * cosines(0);
    fk_Jac_physTrig(forward_Jacobian, sines, cosines);
}

void forward_Jacobian(Matrix3f &forward_Jacobian, const vector3f &theta) {
    /*given motor angles, find the Jacobian d(p_leg)/d(motor_angle)
        where p_leg is relative leg position array of [forward, out, up].
    theta(0) is the hip outward motor angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta(1) is the hip forward motor angle, zero when the leg is pointing down, positive forwards
    theta(2) is the knee angle, zero when the knee is straight, positive forwards
    down is towards the ground, forward and out are body orientation, all 3 are perpendicular to each other
    
    using both forward_Jacobian() and forward_kinematics() is slower than forward_kinematics() with jacobian
    
    sets Jacobian as 3x3 matrix*/
    Vector3f sines = theta.array().sin();
    Vector3f cosines = theta.array().cos();
    fk_Jac_physTrig(Jacobian, sines, cosines);
}

void inverse_Jacobian(Matrix3f &inverse_Jacobian, const Matrix3f &forward_Jacobian, const vector3f &theta) {
    /*find the inverse of the fk Jacobian. Deals with singularity.
    Near theta2=0, Jacobian gets an added term that moves leg forward
    fk_Jac is the forward jacobian, angle is the associated leg angles*/
    float max_width = 0.2, crit_width = 0.02; // the width in radians of the singularity handling
    float width = fabs(theta(2));
    if (width > max_width) {
        inverse_Jacobian = forward_Jacobian.inverse();
    } else { //if we're close to the singularity
        Matrix3f backup;
        backup.block<2,3>(0,0) = pseudoinverse(forward_Jacobian.block<3,2>(0,0));
        backup.block<1,3>(2,0) = 2 * backup.block<1,3>(1,0);
        //TODO encourage backwards knee
        if (width > crit_width) {
            inverse_Jacobian = (width / max_width) * forward_Jacobian.inverse()
                                + (1. - width / max_width) * backup;
        } else { // too close to singularity for any inverse
            inverse_Jacobian = backup;
        }
    }
}

/*def fk_Jacobian_dot(theta, omega):
    """given leg angles and angular velocity, find the time derivative of the Jacobian returned by fk_Jacobian.
    theta(0) is the hip outward motor angle, zero when the leg is pointing down, positive outwards from the side of the body.
    theta(1) is the hip forward motor angle, zero when the leg is pointing down, positive forwards
    theta(2) is the knee angle, zero when the knee is straight, positive forwards
    
    returns 3x3 time derivative of Jacobian matrix"""
    #TODO? complicated, so numerically calculated by motionController instead. */
    

void fk_Jac_physTrig(Matrix3f &Jac, const vector3f &sines, const vector3f &cosines) {
    /*given ratios of physical coordinates, find d(p_leg)/d(motor_angle)
    theta0 is the motor angle outwards
    phi1 is angle between thigh and down in the plane of the leg
    phi2 is the angle between shin and down in the plane of the leg
    sines and cosines are sin and cos of [theta0, phi1, phi2]
    
    returns Jacobian as a 3x3 matrix */
    Jac(FORWARD_IDX,0) = -(sines(1) + sines(2)) * sines(0);
    Jac(FORWARD_IDX,1) = cosines(1) * cosines(0);
    Jac(FORWARD_IDX,2) = cosines(2) * cosines(0);
    Jac(RIGHT_IDX,0) = (cosines(1) + cosines(2)) * cosines(0);
    Jac(RIGHT_IDX,1) = -sines(1) * sines(0);
    Jac(RIGHT_IDX,2) = -sines(2) * sines(0);
    Jac(UP_IDX,0) = (cosines(1) + cosines(2)) * sines(0);
    Jac(UP_IDX,1) = sines(1) * cosines(0);
    Jac(UP_IDX,2) = sines(2) * cosines(0);
}
    

    
    
