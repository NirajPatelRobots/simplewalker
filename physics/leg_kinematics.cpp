
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

void forward_kinematics_R(Matrix3f &R, const Vector3f &theta) {
    // rotate by theta2 around RIGHT axis, then rotate by theta0 around FORWARD axis
    R = Eigen::AngleAxisf(theta(0), FORWARD_DIR) * Eigen::AngleAxisf(theta(2), -LEFT_DIR);
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


float calc_foot_spring_angle(Vector3f &sprung_point, const Vector3f &unsprung_point,
                             const Matrix3f &R, float foot_radius) {
    if (sprung_point(UP_IDX) > foot_radius) {
        return 0; // no reason to bend above ground
    }
    const Vector3f p_i = (sprung_point - unsprung_point);
    const float L = p_i.norm();
    const Vector3f p_l = L * R * UP_DIR;  // vector length L from unsprung_point in the direction of the leg

    const float depth = foot_radius - sprung_point(2);
    const float initial_descent = - p_i(2);
    float angle = 2 * std::atan2(p_l(2) - std::sqrt(p_l(2) * p_l(2) + 2 * depth * initial_descent - depth * depth),
                                 depth - 2 * initial_descent);
    const float singularity_distance = abs(depth - 2 * initial_descent);
    if (singularity_distance < 1e-2) {
        const float backup_angle = 2 * std::atan2(-p_i(2), p_l(2));
        cout << "~~~We're in the calc_foot_spring_angle SINGULARITY ZONE~~~\n"
             << angle << " | " << backup_angle << std::endl;
        // TODO use backup_angle
    }
    if (angle > M_PI)
        angle -= 2 * M_PI;
    if (angle < -M_PI)
        angle += 2 * M_PI;
    sprung_point += p_l * std::sin(angle) + p_i * (std::cos(angle) - 1);
    return angle;
}
