/* utilities for testing sections of the simplewalker code to prove functionality 
TODO:
    typedef for jacobiantest functions?
    statistic for magnitude of jacobian out change over in change (not error, just interesting)
    Verify Matrices are inverses
    use process clock for timing
    JacobianTest subclass of inputoutput_vector_compare
    set input magnitude
    vector_statistic
    scalar_statistic unit
    scalar_statistic std_dev could be nan

Niraj, May 2022*/
#include <iostream>
#include <chrono>
#include <string>
#include "logger.hpp"
#include "leg_kinematics.hpp"

using Eigen::Array3f, Eigen::MatrixXf, Eigen::VectorXf;

struct scalar_statistic {
    float mean{0};
    float max {0};
    float std_dev{0};
    float most_recent{0};
    unsigned num_data{0};
    unsigned num_bad{0};
    std::string unit{};
    bool update(float new_val);  // returns whether statistics were updated, false if invalid input
    bool is_max() const {return (most_recent == max);}
};

std::ostream& operator<<(std::ostream& os, const scalar_statistic& data);

/*struct vector_statistic{ //TODO
    Vector3f mean{0, 0, 0};
    Vector3f max{0, 0, 0};
    Matrix3f covariance{Matrix3f::Identity()};
    Vector3f most_recent{0, 0, 0};
    unsigned num_data{0};
    unsigned num_bad{0};
    bool update(Vector3f new_val);
    bool is_max() const {return (most_recent == max);}
};*/

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

    calculates results as scalar_statistic:
    error_mag, ||jac_output - true_output|| / ||true_output - jac_calc_point||
        the magnitude of prediction error divided by the true distance of the peturbation in the output
    error_angle, dot(unit(jac_output - jac_calc_point), unit(true_output - jac_calc_point))
        the angle between the direction the true output moved after the perturbation and the direction the jacobian output moved
    */
    void (*jac_calc_func)(Matrix3f &jacobian, const Vector3f &input, const Vector3f &other_input);
    void (*true_ref_function)(Vector3f &output, const Vector3f &input, const Vector3f &other_input);
    scalar_statistic jacCalcTime_us_, refCalcTime_us_, output_error_, error_mag_, error_angle_;
    void start_timing(void);
    std::chrono::time_point<std::chrono::steady_clock> starttime;
    unsigned elapsed_time(void);
    void calcAndTimeTrue(Vector3f &output, const Vector3f &input, const Vector3f &other_input);
    void calcAndTimeJac(Matrix3f &jacobian, const Vector3f &input, const Vector3f &other_input);
    void update_results(const Vector3f true_out, const Vector3f approx_out, const Vector3f alt_input,
                        const Vector3f base_out, const Vector3f input_base, const Vector3f input_diff);
    void error_mag_angle_results(const Vector3f &true_out, const Vector3f &approx_out, const Vector3f &base_out);
    void updateFailures(void);
    int failures;
public:
    int num_Jac_samples, num_diff_samples;
    float max_diff, failure_frac_thresh;
    Array3f input_mean, input_max_change, alt_input_mean, alt_input_max_change;
    Vector3f worst_mag_input, worst_angle_input;
    Logger logger;
    JacobianTest();
    void load_settings(const WalkerSettings &settings, string groupname);
    void initResults(void);
    void start_file_logging(string name);
    int run(void (*jac_calc_func)(Matrix3f &jacobian, const Vector3f &input, const Vector3f &other_input),
            void (*true_ref_function)(Vector3f &output, const Vector3f &input, const Vector3f &other_input));
    void print(std::string name, std::string shortname);
    const scalar_statistic &jacCalcTime_us, &refCalcTime_us, &output_error, &error_mag, &error_angle;
};



