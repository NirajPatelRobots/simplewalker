/* simplewalker state for state estimation 
March 2022 blame Niraj
?*/
#include "robot_state.hpp"

DynamicPose::DynamicPose()
: R_(DEFAULT_ROTATION), vect(VectorXf::Zero(N)), acceleration(Vector3f::Zero()), timestamp_us(), R(R_) {
    set_R_(R);
    setAxisFromR_();
}

/* if axis has changed enough, update rotation matrix.*/
void DynamicPose::calculate() {
    float delta_angle = (axis() - R_cached_axis).norm();
    float axis_angle = axis().norm();
    if (axis_angle > 2 * M_PI) {
        axis() -= 2 * M_PI * axis() / axis_angle;
        axis_angle -= 2 * M_PI;
    }
    if (delta_angle > rot_angle_update_thresh) {
        cached_rotation_count = 0;
        set_R_(axis_angle, axis() / axis_angle);
    } else ++cached_rotation_count;
}

void DynamicPose::set_R(const Eigen::Ref<const Matrix3f> &new_R) {
    if ((new_R * new_R.transpose() - Matrix3f::Identity()).norm() > 1e-5 ) {
        std::cerr << "Err: attempted to set non-orthogonal R\n";
        throw std::runtime_error("ERR: attempted to set non-orthogonal R");
    }
    set_R_(new_R);
}

void DynamicPose::increment_R_(float delta_angle, const Vector3f &axis_normalized) {
    set_R_(Eigen::AngleAxisf(delta_angle, axis_normalized) * R);
}

void DynamicPose::set_R_(float angle, const Vector3f &axis_normalized) {
    set_R_(Eigen::AngleAxisf(angle, axis_normalized).toRotationMatrix());
}

void DynamicPose::set_R_(const Eigen::Ref<const Matrix3f> &new_R) {
    R_ = new_R;
    R_cached_axis = axis();
}

void DynamicPose::setAxisFromR_() {
    Eigen::AngleAxisf angle_axis{R};
    axis() = angle_axis.axis() * angle_axis.angle();
    R_cached_axis = axis();
}

//Block3f DynamicPose::pos() {return vect.segment<3>(IDX_POS);}
//const Vector3f DynamicPose::pos() const {return vect.segment<3>(IDX_POS);}
//Block3f DynamicPose::axis() {return vect.segment<3>(IDX_AXIS);}
//const Vector3f DynamicPose::axis() const {return vect.segment<3>(IDX_AXIS);}
//Block3f DynamicPose::vel() {return vect.segment<3>(IDX_VEL);}
//const Vector3f DynamicPose::vel() const {return vect.segment<3>(IDX_VEL);}
//Block3f DynamicPose::angvel() {return vect.segment<3>(IDX_ANGVEL);}
//const Vector3f DynamicPose::angvel() const {return vect.segment<3>(IDX_ANGVEL);}


FootInfo::FootInfo(FootShape polygon_, Vector3f attach_point_, float radius_)
        : points(std::move(polygon_)), attach_point(std::move(attach_point_)), radius(radius_), spring_torque_per_rad() {}

FootInfo::FootInfo(FootShape polygon_, Vector3f attach_point_, float radius_, float spring_k)
        : points(std::move(polygon_)), attach_point(std::move(attach_point_)), radius(radius_), spring_torque_per_rad(spring_k)
{}

FootState::FootState() {
    position.setZero();
    R.setIdentity();
    for (Vector3f &point : points)
        point.setZero();
}

void FootState::set(const Eigen::Ref<const Vector3f> &body_pos, const Eigen::Ref<const Vector3f> &leg_angles,
                    const Matrix3f &R_body, const FootInfo &info, int32_t timestamp_us_) {
    timestamp_us = timestamp_us_;
    Vector3f p_leg;
    Matrix3f R_leg;
    forward_kinematics(p_leg, leg_angles);
    forward_kinematics_R(R_leg, leg_angles);
    position = body_pos + R_body * (info.attach_point + p_leg);
    R = R_body * R_leg;
    for (unsigned long i = 0; i < points.size(); i++) {
        points[i] = R * info.points[i] + position;
    }
    if (info.spring_torque_per_rad.has_value()) {
        if (points_contact[1]) { // TODO determine which points are sprung without hard-coding (points_are_sprung?)
            spring_angle = calc_foot_spring_angle(points[1], points[0], R, info.radius);
            points[2] = points[0] + (points[0] - points[1]);
        }else if (points_contact[2]) {
            spring_angle = calc_foot_spring_angle(points[2], points[0], R, info.radius);
            points[1] = points[0] + (points[0] - points[2]);
        }
    }
    for (unsigned long i = 0; i < points.size(); i++) {
        points_contact[i] = points[i](UP_IDX) < info.radius;
    }
}

bool FootState::in_contact() const {
    return std::any_of(points_contact.begin(), points_contact.end(), [](bool b) {return b;});
}


RobotState::RobotState() : DynamicPose(), foot_states() {}
