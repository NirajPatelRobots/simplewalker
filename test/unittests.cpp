/* Test parts of simplewalker code to prove functionality

Niraj, June 2022 */
#include <memory>
#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>
#include "test_utils.hpp"


void fk_jac_forTest(Matrix3f &jacobian, const Vector3f &input) {
    forward_Jacobian(jacobian, input);
}
void fk_jac_true_ref(Vector3f &output, const Vector3f &input) {
    forward_kinematics(output, input);
}


int main(void) {
    int jacobian_samples {1000}, differential_samples {1000};
    float max_differential_change {0.1};
    auto jacobianTest = std::make_unique<JacobianTest>(jacobian_samples, differential_samples, max_differential_change);

    // test forward kinematics
    float theta_mean{0}, theta_max{M_PI_2};
    jacobianTest->run(fk_jac_forTest, fk_jac_true_ref, theta_mean, theta_max);
    std::cout<<"Forward kinematics Jacobian: max_differential_change = "<<max_differential_change<<std::endl
             <<"FK Jac output error:"<<std::endl<<jacobianTest->output_error
             <<"FK Jac fractional magnitude error:"<<std::endl<<jacobianTest->error_mag
             <<"FK Jacobian error angle [rad]:"<<std::endl<<jacobianTest->error_angle
             <<"Jacobian Calculation time [us]:"<<std::endl<<jacobianTest->jacCalcTime_us
             <<"True Function Calculation time [us]:"<<std::endl<<jacobianTest->refCalcTime_us<<std::endl;

    return 0;
}
