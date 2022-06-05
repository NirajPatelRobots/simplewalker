/* utilities for testing sections of the simplewalker code to prove functionality 
TODO:
    typedef for jacobiantest functions?
    std_dev could be nan
    statistic for magnitude of out change over in change (not error, just interesting)

Niraj, May 2022*/
#include "leg_kinematics.hpp"
#include <cmath>
#include <iostream>
#include <chrono>

using Eigen::Array3f, Eigen::MatrixXf, Eigen::VectorXf;

template<typename T>
struct run_data {
    T mean;
    T max;
    T std_dev;
    unsigned num_data;
    unsigned num_bad;
    run_data() : mean(T{}), max(T{}), std_dev(T{}), num_data(0), num_bad(0) {};
};

void update_run_data(run_data<float> &data, float new_val);

template<typename T>
std::ostream& operator<<(std::ostream& os, const run_data<T>& data) {
    os<<"Mean: "<<data.mean<<", max: "<<data.max<<", std_dev: "<<data.std_dev<<" / "<<data.num_data<<" elements";
    if (data.num_bad) 
        os<<" ("<<data.num_bad<<" bad)";
    os<<std::endl;
    return os;
}


struct single_Jac_run_result {
    float error; //the magnitude of difference between jacobian calculated and true output
    float input_delta; //mag of how much the input changed
    float output_delta; //mag of how much the true output changed
    float input[3];
};


class JacobianTest {
    /* class to test how well a calculated Jacobian J actually matches its intended behavior.
    run()s nested loops which first calculates the Jacobian at a random input. It perturbs the input by a small differential,
        then compares the true output of the input with the version of the output calculated with the jacobian.

    This class takes two functions as arguments.
    true_ref_function calculates the true value of the equation output = f(input).
    The test jacobian J is defined as the Jacobian of the output with respect to
        the input at that value of the input.
    jac_calc_func calculates the jacobian J given the value of the input.
    num_Jac_samples is how many different values of the jacobian to calculate.
    num_differential_samples is how many variations of the input are used to evaluate each jacobian.
    input_mean is the mean value of the input. input_max_change is the maximum + or - difference from that for calculating a jacobian.
    max_diff is the maximum change in the differential input vector used to evaluate the jacobian's linearization.

    calculates results as run_data<float>:
    error_mag, ||jac_output - true_output|| / ||true_output - jac_calc_point||
        the magnitude of prediction error divided by the true distance of the peturbation in the output
    error_angle, dot(unit(jac_output - jac_calc_point), unit(true_output - jac_calc_point))
        the angle between the direction the true output moved after the perturbation and the direction the jacobian output moved
    */
    void (*jac_calc_func)(Matrix3f &jacobian, const Vector3f &input);
    void (*true_ref_function)(Vector3f &output, const Vector3f &input);
    run_data<float> jacCalcTime_us_, refCalcTime_us_, output_error_, error_mag_, error_angle_;
    VectorXf singleRunResults; // TODO: Do a linear regression on this to get error as fcn of all other results
public:
    int num_Jac_samples, num_diff_samples;
    float max_diff;
    Array3f input_mean, input_max_change;
    JacobianTest(int jacobian_samples, int differential_samples, float max_differential_change);
    int run(void (*jac_calc_func)(Matrix3f&, const Vector3f&), void (*true_ref_function)(Vector3f&, const Vector3f&),
               float input_mean_val, float input_max_change_val);
    //results
    const run_data<float> &jacCalcTime_us, &refCalcTime_us, &output_error, &error_mag, &error_angle;
};
