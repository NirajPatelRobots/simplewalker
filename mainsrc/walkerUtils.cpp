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
void Jac_rotated_wrt_axis_angle(Eigen::Ref<Matrix3f> Jacobian, const Vector3f &axis, const Vector3f &vec) {
    // Buckle up cowboys because this is a whole mess of an equation
    // TODO BUG
    float angle = axis.norm();
    Vector3f first_part{(cosf(angle) - sinf(angle)/angle) * axis.cross(vec) * powf(angle, -2)
                        - sinf(angle) * vec
                        + axis.dot(vec) * powf(angle, -2) * (sinf(angle) + 2*(cosf(angle)-1) / angle) * axis};
    Matrix3f raised_vec {{0, -vec(2), vec(1)}, {vec(2), 0, -vec(0)}, {-vec(1), vec(0), 0}};
    Jacobian = (first_part * axis.transpose() - sinf(angle) * raised_vec
                +(1 - cosf(angle)) / angle * (axis * vec.transpose() + axis.dot(vec) * Matrix3f::Identity())) / angle;
}


void jacobian_unitvec_wrt_vec(Matrix3f &jacobian, const Vector3f &vec) {
    float norm = vec.norm();
    jacobian = Matrix3f::Identity() / norm - vec * vec.transpose() * powf(norm, -3);
}
