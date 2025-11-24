#include "test_utils.hpp"
namespace chrono = std::chrono;

JacobianTest::JacobianTest(void)
    : jac_calc_func(NULL), true_ref_function(NULL),
    num_Jac_samples(100), num_diff_samples(100), max_diff(0.01),
    failure_frac_thresh(0.05), input_mean(Vector3f::Zero()), input_max_change(Vector3f::Constant(M_PI)),
    logger(), jacCalcTime_us(jacCalcTime_us_), refCalcTime_us(refCalcTime_us_),
    output_error(output_error_), error_mag(error_mag_), error_angle(error_angle_)
    {
    initResults();
}

void JacobianTest::load_settings(const WalkerSettings &settings, string name) {
    const char * groupname = name.c_str();
    max_diff = settings.f(groupname, "max_diff");
    string file_tag = settings.cstr(groupname, "log_file_tag");
    if (file_tag.length() > 0) {
        logger = Logger("data/jacobian_test_" + file_tag + ".log"); }
    else
        logger = Logger();
    std::vector<float> input_mean_vec = settings.vf(groupname, "input_mean");
    if (input_mean_vec.size() == 3)     input_mean = Eigen::Map<Array3f>(input_mean_vec.data());
    else                                input_mean = Array3f::Zero();
    if (settings.b(groupname, "alt_input_mean")) {
        input_mean_vec = settings.vf(groupname, "alt_input_mean");
    }
    if (input_mean_vec.size() == 3)     alt_input_mean = Eigen::Map<Array3f>(input_mean_vec.data());
    else                                alt_input_mean = input_mean;

    input_max_change = Array3f::Constant(settings.f(groupname, "input_max_change_mag") / sqrt(3));
    if (settings.b(groupname, "alt_input_max_change_mag")) {
        alt_input_max_change = Array3f::Constant(settings.f(groupname, "alt_input_max_change_mag") / sqrt(3));
    }
    else alt_input_max_change = input_max_change;
    num_Jac_samples = settings.i(groupname, "num_Jac_samples");
    num_diff_samples = settings.i(groupname, "num_diff_samples");
    failure_frac_thresh = settings.f(groupname, "failure_frac_thresh");
}

void JacobianTest::initResults(void) {
    jacCalcTime_us_ = scalar_statistic();
    refCalcTime_us_ = scalar_statistic();
    output_error_ = scalar_statistic();
    error_mag_ = scalar_statistic();
    error_angle_ = scalar_statistic();
    failures = 0;
}

void printInputAndResult(const Vector3f &input, const scalar_statistic &result) {
    std::cout<< result.most_recent<<" "<<result.unit<<" ["<<input.transpose() / M_PI
            <<"]pi   norm: "<< input.norm() / M_PI <<"pi  basisness:"
            << input.lpNorm<Eigen::Infinity>() / input.norm() <<std::endl;
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
        alt_input = alt_input_mean + Array3f::Random() * alt_input_max_change;
        calcAndTimeJac(this_jac, input, alt_input);
        calcAndTimeTrue(base_out, input, alt_input);
        for (int n_dif = 0; n_dif < num_diff_samples; ++n_dif) {
            diff = Array3f::Random() * max_diff;
            approx_out = base_out + this_jac * diff;
            calcAndTimeTrue(true_out, input + diff, alt_input);
            update_results(true_out, approx_out, base_out, alt_input, input, diff);
        }
    }
    logger = Logger();
    return failures;
}


void JacobianTest::calcAndTimeTrue(Vector3f &output, const Vector3f &input, const Vector3f &other_input) {
    start_timing();
    true_ref_function(output, input, other_input);
    refCalcTime_us_.update(elapsed_time());
}

void JacobianTest::calcAndTimeJac(Matrix3f &jacobian, const Vector3f &input, const Vector3f &other_input) {
    start_timing();
    jac_calc_func(jacobian, input, other_input);
    jacCalcTime_us_.update(elapsed_time());
}

void JacobianTest::start_timing(void) { starttime = chrono::steady_clock::now(); }

unsigned JacobianTest::elapsed_time(void) {
    return chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - starttime).count();
}

void JacobianTest::print(std::string name, std::string shortname) {
    std::cout<<name <<" Jacobian Test"<< endl
             <<output_error.num_data<<" tries "<< failures <<" failures (>" << failure_frac_thresh << ")" << endl
             <<"max_differential_change = "<< max_diff <<std::endl
             <<"Input mean: ["<<input_mean.transpose()<<"] input max change: ["<<input_max_change.transpose()<<"]"<<std::endl
             <<"Alt input mean: ["<<alt_input_mean.transpose()<<"] alt input max change: ["<<alt_input_max_change.transpose()<<"]"<<std::endl
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

void updateWorstInput(Vector3f &worst_input_save, const Vector3f &input, const scalar_statistic &result_stat) {
    if (result_stat.is_max()) {
        worst_input_save = input;
        // printInputAndResult(worst_input_save, result_stat);
    }
}

void JacobianTest::updateFailures(void) {
    if (error_mag.most_recent > failure_frac_thresh || error_angle.most_recent > failure_frac_thresh) 
        {failures++;}
}

void JacobianTest::update_results(const Vector3f true_out, const Vector3f approx_out,
                                  const Vector3f base_out, const Vector3f alt_input,
                                  const Vector3f input_base, const Vector3f input_diff) {
    error_mag_angle_results(true_out, approx_out, base_out);
    updateWorstInput(worst_mag_input, input_base + input_diff, error_mag);
    updateWorstInput(worst_angle_input, input_base + input_diff, error_angle);
    updateFailures();
    if (logger.get_filename().length() > 0) {
        logger.log("input", input_base + input_diff);
        logger.log("input_diff", input_diff);
        logger.log("alt_input", alt_input);
        logger.log("base_out", base_out);
        logger.log("true_out", true_out);
        logger.log("approx_out", approx_out);
        logger.print();
    }
}

bool scalar_statistic::update(float new_val) {
    if (!std::isfinite(new_val)) {
        num_bad++;
        return false;
    }
    unsigned n = num_data++;
    float new_mean = (mean * n + new_val)/(n + 1);
    if (new_val > max)  max = new_val;
    std_dev = sqrtf((n * powf(std_dev, 2) + n*(n-1) * powf(new_mean - mean, 2)) / (n+1) );
    mean = new_mean;
    most_recent = new_val;
    return true;
}

std::ostream& operator<<(std::ostream& os, const scalar_statistic& data) {
    os<<"Mean: "<<data.mean<<", max: "<<data.max<<", std_dev: "<<data.std_dev<<" [n = "<<data.num_data<<"]";
    if (data.num_bad)
        os<<" ("<<data.num_bad<<" bad)";
    os<<std::endl;
    return os;
}
