#include "test_utils.hpp"
namespace chrono = std::chrono;

JacobianTest::JacobianTest(int jacobian_samples, int differential_samples, float max_differential_change)
    : jac_calc_func(NULL), true_ref_function(NULL),
    num_Jac_samples(jacobian_samples), num_diff_samples(differential_samples), max_diff(max_differential_change),
    jacCalcTime_us(jacCalcTime_us_), refCalcTime_us(refCalcTime_us_),
    output_error(output_error_), error_mag(error_mag_), error_angle(error_angle_)
{}

int JacobianTest::run(void (*jac_calc_func)(Matrix3f&, const Vector3f&),
                void (*true_ref_function)(Vector3f&, const Vector3f&),
                float input_mean_val, float input_max_change_val) {
    this->jac_calc_func = jac_calc_func;
    this->true_ref_function = true_ref_function;
    input_mean = Array3f::Constant(1) * input_mean_val;
    input_max_change = Array3f::Constant(1) * input_max_change_val;
    jacCalcTime_us_ = run_data<float>();
    refCalcTime_us_ = run_data<float>();
    output_error_ = run_data<float>();
    error_mag_ = run_data<float>();
    error_angle_ = run_data<float>();
    //singleRunResults = VectorXf(num_Jac_samples * num_diff_samples);
    Matrix3f this_jac;
    Vector3f input, diff, base_out, diff_out, approx_out, true_out, output_diff;
    chrono::time_point<chrono::steady_clock> starttime;
    chrono::microseconds elapsed_time;
    for (int n_jac = 0; n_jac < num_Jac_samples; ++n_jac) {
        input = input_mean + Array3f::Random() * input_max_change;
            starttime = chrono::steady_clock::now();
        jac_calc_func(this_jac, input);
            elapsed_time = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - starttime);
            update_run_data(jacCalcTime_us_, elapsed_time.count());
            starttime = chrono::steady_clock::now();
        true_ref_function(base_out, input);
            elapsed_time = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - starttime);
            update_run_data(refCalcTime_us_, elapsed_time.count());
        for (int n_dif = 0; n_dif < num_diff_samples; ++n_dif) {
            diff = Array3f::Random() * max_diff;
            diff_out = this_jac * diff;
            approx_out = base_out + diff_out;
                starttime = chrono::steady_clock::now();
            true_ref_function(true_out, input + diff);
                elapsed_time = chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - starttime);
                update_run_data(refCalcTime_us_, elapsed_time.count());
            float scalar_error = (approx_out - true_out).norm();
            update_run_data(output_error_, scalar_error);
            output_diff = true_out - base_out;
            update_run_data(error_mag_, scalar_error / output_diff.norm());
            diff_out.normalize();
            output_diff.normalize();
            update_run_data(error_angle_, acos(diff_out.dot(output_diff)));
        }
    }
    return 0;
}


void update_run_data(run_data<float> &data, float new_val) {
    if (!std::isfinite(new_val)) {
        data.num_bad++;
        return;
    }
    unsigned n = data.num_data++;
    float new_mean = (data.mean * n + new_val)/(n + 1);
    if (new_val > data.max) data.max = new_val;
    data.std_dev = sqrtf((n * powf(data.std_dev, 2) + n*(n-1) * powf(new_mean - data.mean, 2)) / (n+1) );
    data.mean = new_mean;
}
