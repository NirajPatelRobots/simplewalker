#include "rotation.hpp"


// rotation
void Jac_rotated_wrt_axis_angle(Eigen::Ref<Matrix3f> Jacobian, const Vector3f &axis_angle,
                                       const Matrix3f &R, const Vector3f &unrotated_vector) {
    // from A compact formula for the derivative of a 3-D rotation in exponential coordinates
    // Guillermo Gallego, Anthony Yezzi 2014 https://arxiv.org/pdf/1312.0788.pdf
    Jacobian = - R * raised_cross_matrix(unrotated_vector) / axis_angle.squaredNorm()
               * (axis_angle * axis_angle.transpose()
                  + (R.transpose() - Matrix3f::Identity()) * raised_cross_matrix(axis_angle));
}

Matrix3f axis_angle_to_R(const Vector3f &axis_angle) {
    float angle = axis_angle.norm();
    return Eigen::AngleAxisf(angle, axis_angle / angle).toRotationMatrix();
}

Matrix3f raised_cross_matrix(const Vector3f &vec) {
    return Matrix3f{{0, -vec(2), vec(1)}, {vec(2), 0, -vec(0)}, {-vec(1), vec(0), 0}};
}

void jacobian_unitvec_wrt_vec(Matrix3f &jacobian, const Vector3f &vec) {
    float norm = vec.norm();
    jacobian = Matrix3f::Identity() / norm - vec * vec.transpose() * powf(norm, -3);
}
