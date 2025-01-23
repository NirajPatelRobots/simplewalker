#include "walkerUtils.hpp"

bool scalar_statistic::update(float new_val) {
    if (!std::isfinite(new_val)) {
        num_bad++;
        return false;
    }
    unsigned n = num_data++;
    float new_mean = (mean * n + new_val)/(n + 1);
    if (new_val > max)  max = new_val;
    std_dev = sqrtf((n * powf(std_dev, 2) + n*(n-1) * powf(new_mean - mean, 2)) / (n+1) );
    mean = new_mean;
    most_recent = new_val;
    return true;
}

std::ostream& operator<<(std::ostream& os, const scalar_statistic& data) {
    os<<"Mean: "<<data.mean<<", max: "<<data.max<<", std_dev: "<<data.std_dev<<" / "<<data.num_data<<" elements";
    if (data.num_bad) 
        os<<" ("<<data.num_bad<<" bad)";
    os<<std::endl;
    return os;
}

// rotation
void Jac_rotated_wrt_axis_angle(Eigen::Ref<Matrix3f> Jacobian, const Vector3f &axis_angle,
                                       const Matrix3f &R, const Vector3f &unrotated_vector) {
    // from A compact formula for the derivative of a 3-D rotation in exponential coordinates
    // Guillermo Gallego, Anthony Yezzi 2014 https://arxiv.org/pdf/1312.0788.pdf
    float angle = axis_angle.norm();
    Jacobian = - R * raised_cross_matrix(unrotated_vector) / (angle * angle)
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
