#include "test_utils.hpp"
namespace chrono = std::chrono;

JacobianTest::JacobianTest(int jacobian_samples, int differential_samples, float max_differential_change)
    : jac_calc_func(NULL), true_ref_function(NULL),
    num_Jac_samples(jacobian_samples), num_diff_samples(differential_samples), max_diff(max_differential_change),
    failure_percent_thresh(0.05), input_mean(Vector3f::Zero()), input_max_change(Vector3f::Constant(M_PI)),
    jacCalcTime_us(jacCalcTime_us_), refCalcTime_us(refCalcTime_us_),
    output_error(output_error_), error_mag(error_mag_), error_angle(error_angle_)
    {
    initResults();
}

void JacobianTest::initResults(void) {
    jacCalcTime_us_ = scalar_statistic();
    refCalcTime_us_ = scalar_statistic();
    output_error_ = scalar_statistic();
    error_mag_ = scalar_statistic();
    error_angle_ = scalar_statistic();
    failures = 0;
    //singleRunResults = VectorXf(num_Jac_samples * num_diff_samples);
}

void printInputAndResult(const Vector3f &input, const scalar_statistic &result) {
    std::cout<< result.most_recent<<" "<<result.unit<<" ["<<input.transpose() / M_PI
            <<"]pi   norm: "<< input.norm() / M_PI <<"pi  basisness:"
            << input.lpNorm<Eigen::Infinity>() / input.norm() <<std::endl;
}

void updateWorstInput(Vector3f &worst_input_save, const Vector3f &input, const scalar_statistic &result_stat) {
    if (result_stat.is_max()) {
        worst_input_save = input;
        printInputAndResult(worst_input_save, result_stat);
    }
}

int JacobianTest::run(void (*jac_calc_func)(Matrix3f&, const Vector3f&, const Vector3f&),
                      void (*true_ref_function)(Vector3f&, const Vector3f&, const Vector3f&)){
    this->jac_calc_func = jac_calc_func;
    this->true_ref_function = true_ref_function;
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
            error_mag_angle_results(true_out, approx_out, base_out);
            updateWorstInput(worst_mag_input, input + diff, error_mag);
            updateWorstInput(worst_angle_input, input + diff, error_angle);
            updateFailures();
        }
    }
    return failures;
}

void JacobianTest::updateFailures(void) {
    if (error_mag.most_recent > failure_percent_thresh || error_angle.most_recent > failure_percent_thresh) 
        {failures++;}
}

void JacobianTest::start_timing(void) {
    starttime = chrono::steady_clock::now();
}

unsigned JacobianTest::elapsed_time(void) {
    return chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - starttime).count();
}

void JacobianTest::print(std::string name, std::string shortname) {
    std::cout<<name<<" Jacobian Test  | "<<refCalcTime_us.num_data<<" tries "<<failures<<" failures"<<std::endl
             <<"max_differential_change = "<< max_diff <<std::endl
             <<"Input mean: ["<<input_mean.transpose()<<"] input max change: ["<<input_max_change.transpose()<<"]"<<std::endl
             <<shortname<<" jac output error:"<<std::endl<< output_error
             <<shortname<<" jac fractional magnitude error:"<<std::endl<< error_mag
             <<shortname<<" jac error angle [rad]:"<<std::endl<< error_angle
             <<shortname<<" Jacobian Calculation time [us]:"<<std::endl<< jacCalcTime_us
             <<shortname<<" True Function Calculation time [us]:"<<std::endl<< refCalcTime_us
             <<shortname<<" worst magnitude input: ["<< worst_mag_input.transpose() <<"]"<<std::endl
             <<shortname<<" worst direction input: ["<< worst_angle_input.transpose() <<"]"<<std::endl<<std::endl;
}

void JacobianTest::error_mag_angle_results(const Vector3f &true_out, const Vector3f &approx_out, const Vector3f &base_out) {
    //std::cout<<"Base "<<base_out.transpose()<<" True "<<true_out.transpose()<<" Approx "<<approx_out.transpose()<<std::endl;                                
    float new_output_error = (approx_out - true_out).norm();
    output_error_.update(new_output_error);
    Vector3f true_diff = true_out - base_out;
    Vector3f approx_diff = approx_out - base_out;
    error_mag_.update(new_output_error / true_diff.norm());
    approx_diff.normalize();
    true_diff.normalize();
    error_angle_.update(acos(approx_diff.dot(true_diff)));
}



