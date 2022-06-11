#include "test_utils.hpp"
namespace chrono = std::chrono;

JacobianTest::JacobianTest(int jacobian_samples, int differential_samples, float max_differential_change)
    : jac_calc_func(NULL), true_ref_function(NULL),
    num_Jac_samples(jacobian_samples), num_diff_samples(differential_samples), max_diff(max_differential_change),
    jacCalcTime_us(jacCalcTime_us_), refCalcTime_us(refCalcTime_us_),
    output_error(output_error_), error_mag(error_mag_), error_angle(error_angle_) 
    {
    initResults();
}

void JacobianTest::initResults(void) {
    jacCalcTime_us_ = scalar_result();
    refCalcTime_us_ = scalar_result();
    output_error_ = scalar_result();
    error_mag_ = scalar_result();
    error_angle_ = scalar_result();
    //singleRunResults = VectorXf(num_Jac_samples * num_diff_samples);
}

int JacobianTest::run(void (*jac_calc_func)(Matrix3f&, const Vector3f&, const Vector3f&),
                void (*true_ref_function)(Vector3f&, const Vector3f&, const Vector3f&),
                float input_mean_val, float input_max_change_val) {
    this->jac_calc_func = jac_calc_func;
    this->true_ref_function = true_ref_function;
    input_mean = Array3f::Constant(1) * input_mean_val;
    input_max_change = Array3f::Constant(1) * input_max_change_val;
    initResults();
    Matrix3f this_jac{Matrix3f::Identity()};
    Vector3f input, alt_input, diff, base_out{0, 0, 0}, approx_out, true_out{1, 0, 0};
    for (int n_jac = 0; n_jac < num_Jac_samples; ++n_jac) {
        input = input_mean + Array3f::Random() * input_max_change;
        alt_input = input_mean + Array3f::Random() * input_max_change;
            start_timing();
        jac_calc_func(this_jac, input, alt_input);
            jacCalcTime_us_.update(elapsed_time());
            start_timing();
        true_ref_function(base_out, input, alt_input);
            refCalcTime_us_.update(elapsed_time());
        for (int n_dif = 0; n_dif < num_diff_samples; ++n_dif) {
            diff = Array3f::Random() * max_diff;
            approx_out = base_out + this_jac * diff;
                start_timing();
            true_ref_function(true_out, input + diff, alt_input);
                refCalcTime_us_.update(elapsed_time());
            error_mag_angle_results(output_error_, error_mag_, error_angle_, true_out, approx_out, base_out);
        }
    }
    return 0;
}

void JacobianTest::start_timing(void) {
    starttime = chrono::steady_clock::now();
}

unsigned JacobianTest::elapsed_time(void) {
    return chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - starttime).count();
}

void JacobianTest::print(std::string name, std::string shortname) {
    std::cout<<name<<" Jacobian max_differential_change = "<< max_diff <<std::endl
             <<shortname<<" jac output error:"<<std::endl<< output_error
             <<shortname<<" jac fractional magnitude error:"<<std::endl<< error_mag
             <<shortname<<" jac error angle [rad]:"<<std::endl<< error_angle
             <<shortname<<" Jacobian Calculation time [us]:"<<std::endl<< jacCalcTime_us
             <<shortname<<" True Function Calculation time [us]:"<<std::endl<< refCalcTime_us <<std::endl;
}


bool scalar_result::update(float new_val) {
    if (!std::isfinite(new_val)) {
        num_bad++;
        return false;
    }
    unsigned n = num_data++;
    float new_mean = (mean * n + new_val)/(n + 1);
    if (new_val > max) max = new_val;
    std_dev = sqrtf((n * powf(std_dev, 2) + n*(n-1) * powf(new_mean - mean, 2)) / (n+1) );
    mean = new_mean;
    return true;
}

std::ostream& operator<<(std::ostream& os, const scalar_result& data) {
    os<<"Mean: "<<data.mean<<", max: "<<data.max<<", std_dev: "<<data.std_dev<<" / "<<data.num_data<<" elements";
    if (data.num_bad) 
        os<<" ("<<data.num_bad<<" bad)";
    os<<std::endl;
    return os;
}

void error_mag_angle_results(scalar_result &scalar_error, scalar_result &error_mag, scalar_result &error_angle, 
                             const Vector3f &true_out, const Vector3f &approx_out, const Vector3f &base_out) {
    //std::cout<<"Base "<<base_out.transpose()<<" True "<<true_out.transpose()<<" Approx "<<approx_out.transpose()<<std::endl;                                
    float new_scalar_error = (approx_out - true_out).norm();
    scalar_error.update(new_scalar_error);
    Vector3f true_diff = true_out - base_out;
    Vector3f approx_diff = approx_out - base_out;
    error_mag.update(new_scalar_error / true_diff.norm());
    approx_diff.normalize();
    true_diff.normalize();
    error_angle.update(acos(approx_diff.dot(true_diff)));
}

