
#include "leg_kinematics.hpp"

void fk_Jac_physTrig(Matrix3f &Jac, const Vector3f &sines, const Vector3f &cosines);

void forward_kinematics(Vector3f &p_leg, const Vector3f &theta) {
    Vector3f sines = theta.array().sin();
    Vector3f cosines = theta.array().cos();
    p_leg(FORWARD_IDX) = (THIGH_LENGTH * sines(1) + SHIN_LENGTH * sines(2)) * cosines(0);
    p_leg(LEFT_IDX) = (THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * sines(0);
    p_leg(UP_IDX) = -(THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * cosines(0);
}

void forward_kinematics(Vector3f &p_leg, Matrix3f &forward_Jacobian, const Vector3f &theta) {
    Vector3f sines = theta.array().sin();
    Vector3f cosines = theta.array().cos();
    p_leg(FORWARD_IDX) = (THIGH_LENGTH * sines(1) + SHIN_LENGTH * sines(2)) * cosines(0);
    p_leg(LEFT_IDX) = (THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * sines(0);
    p_leg(UP_IDX) = -(THIGH_LENGTH * cosines(1) + SHIN_LENGTH * cosines(2)) * cosines(0);
    fk_Jac_physTrig(forward_Jacobian, sines, cosines);
}

void forward_Jacobian(Matrix3f &forward_Jacobian, const Vector3f &theta) {
    Vector3f sines = theta.array().sin();
    Vector3f cosines = theta.array().cos();
    fk_Jac_physTrig(forward_Jacobian, sines, cosines);
}

void inverse_Jacobian(Matrix3f &inverse_Jacobian, const Matrix3f &forward_Jacobian, const Vector3f &theta) {
    float max_width = 0.2, crit_width = 0.02; // the width in radians of the singularity handling
    float width = fabs(theta(2));
    if (width > max_width) {
        inverse_Jacobian = forward_Jacobian.inverse();
    } else { //if we're close to the singularity
        Matrix3f backup;
        backup.block<2,3>(0,0) = forward_Jacobian.block<3,2>(0,0).completeOrthogonalDecomposition().pseudoInverse();
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

void fk_Jac_physTrig(Matrix3f &Jac, const Vector3f &sines, const Vector3f &cosines) {
    /*given ratios of physical coordinates, find d(p_leg)/d(motor_angle)
    theta0 is the motor angle outwards
    phi1 is angle between thigh and down in the plane of the leg
    phi2 is the angle between shin and down in the plane of the leg
    sines and cosines are sin and cos of [theta0, phi1, phi2]
    
    sets Jacobian as a 3x3 matrix */
    Jac(FORWARD_IDX,0) = -(sines(1) + sines(2)) * sines(0);
    Jac(FORWARD_IDX,1) = cosines(1) * cosines(0);
    Jac(FORWARD_IDX,2) = cosines(2) * cosines(0);
    Jac(LEFT_IDX,0) = (cosines(1) + cosines(2)) * cosines(0);
    Jac(LEFT_IDX,1) = -sines(1) * sines(0);
    Jac(LEFT_IDX,2) = -sines(2) * sines(0);
    Jac(UP_IDX,0) = (cosines(1) + cosines(2)) * sines(0);
    Jac(UP_IDX,1) = sines(1) * cosines(0);
    Jac(UP_IDX,2) = sines(2) * cosines(0);
    Jac *= SHIN_LENGTH;
}

