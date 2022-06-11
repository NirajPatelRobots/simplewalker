/* Test parts of simplewalker code to prove functionality
TODO:
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

void body_axis_angle_jac_forTest(Matrix3f &jacobian, const Vector3f &axis_angle, const Vector3f&world_vector) {
    // body_Jac_axis_angle(jacobian, axis_angle, world_vector);
}
void body_axis_angle_jac_true_ref(Vector3f &output, const Vector3f &axis_angle, const Vector3f&world_vector) {
    // output = R(w) * world_vector;
}


int main(void) {
    int jacobian_samples {1000}, differential_samples {1000};
    float max_differential_change {0.1};
    auto jacobianTest {std::make_unique<JacobianTest>(jacobian_samples, differential_samples, max_differential_change)};

    // test forward kinematics
    float theta_mean{0}, theta_max{M_PI_2};
    jacobianTest->run(fk_jac_forTest, fk_jac_true_ref, theta_mean, theta_max);
    jacobianTest->print("Forward Kinematics", "fk");

    // test body frame vector wrt axis-angle rotation vector
    jacobianTest->run(body_axis_angle_jac_forTest, body_axis_angle_jac_true_ref, theta_mean, theta_max);
    jacobianTest->print("Body frame vector axis-angle", "Body frame");
    return 0;
}
