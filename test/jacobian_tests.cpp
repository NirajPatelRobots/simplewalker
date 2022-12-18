/* Test jacobians in simplewalker code to prove functionality
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
    unique_ptr<JacobianTest> jacobianTest {std::make_unique<JacobianTest>()};
    WalkerSettings testSettings("settings/jacobian_test_settings.xml");

    if (!testSettings.b("forward_kinematics_jac", "skip")) {
        jacobianTest->load_settings(testSettings, "forward_kinematics_jac");
        jacobianTest->run(fk_jac_forTest, fk_jac_true_ref);
        jacobianTest->print("Forward Kinematics", "fk");
    }

    if (!testSettings.b("unit_vector_jac", "skip")) {
        jacobianTest->load_settings(testSettings, "unit_vector_jac");
        jacobianTest->run(unit_jac_forTest, unit_jac_true_ref);
        jacobianTest->print("Unit vector wrt vector", "Unit");
    }

    if (!testSettings.b("rotation_axis_angle_jac", "skip")) {
        jacobianTest->load_settings(testSettings, "rotation_axis_angle_jac");
        jacobianTest->run(axis_jac_forTest, axis_angle_jac_true_ref);
        jacobianTest->print("Rotated vector wrt axis-angle", "Rotate");
    }
    return 0;
}
