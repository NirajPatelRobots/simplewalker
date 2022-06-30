/* Test parts of simplewalker code to prove functionality
TODO:
    load settings
Niraj, June 2022 */
#include <memory>
#include <iostream>
#include "test_utils.hpp"


void fk_jac_forTest(Matrix3f &jacobian, const Vector3f &input, const Vector3f&) {
    forward_Jacobian(jacobian, input);
}
void fk_jac_true_ref(Vector3f &output, const Vector3f &input, const Vector3f&) {
    forward_kinematics(output, input);
}

void axis_jac_forTest(Matrix3f &jacobian, const Vector3f &input, const Vector3f&other_input) {
    Jac_rotated_wrt_axis_angle(jacobian, input, other_input);
}
void axis_angle_jac_true_ref(Vector3f &output, const Vector3f &axis_angle, const Vector3f&vector) {
    float angle = axis_angle.norm();
    Vector3f axis = axis_angle / angle;
    output = Eigen::AngleAxisf(angle, axis) * vector;
}

void unit_jac_forTest(Matrix3f &jacobian, const Vector3f &input, const Vector3f&) {
    jacobian_unitvec_wrt_vec(jacobian, input);
}
void unit_jac_true_ref(Vector3f &output, const Vector3f &input, const Vector3f&) {
    output = input;
    output.normalize();
}


int main(void) {
    int jacobian_samples {1000}, differential_samples {1000};
    float max_differential_change {0.1};
    auto jacobianTest {std::make_unique<JacobianTest>(jacobian_samples, differential_samples, max_differential_change)};

    Array3f &theta_mean{jacobianTest->input_mean}, &theta_max_change{jacobianTest->input_max_change};
    theta_mean = Array3f::Constant(0 / sqrt(3));
    //theta_mean(1) *= -1;
    theta_max_change = Array3f::Constant((M_PI - 0.2) / sqrt(3));

    // jacobianTest->run(fk_jac_forTest, fk_jac_true_ref);
    // jacobianTest->print("Forward Kinematics", "fk");

    //jacobianTest->run(unit_jac_forTest, unit_jac_true_ref);
    //jacobianTest->print("Unit vector wrt vector", "Unit");

    jacobianTest->max_diff = 0.001;
    jacobianTest->run(axis_jac_forTest, axis_angle_jac_true_ref);
    jacobianTest->print("Rotated vector wrt axis-angle", "Rotated");
    return 0;
}
