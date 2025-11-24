/* Code for testing state estimator
TODO:
    true_state in more tests, set sensors from true_state
    move parts of tests into parent LocalizationTester
March 2022 */
#include "state_estimation.hpp"
#include "convenientLogger.hpp"
#include "test_utils.hpp"
#include <chrono>
#include <fstream>
#include <random>
namespace chrono = std::chrono;

class LocalizationTester {
public:
    const WalkerSettings settings;
    unique_ptr<SensorData> sensor_data;
    unique_ptr<SensorBoss> sensors;
    unique_ptr<StateEstimator> EKF;
    unique_ptr<RobotState> true_state;
    RobotState &state;
    RobotState &state_pred;
    unique_ptr<ConvenientLogger> logger;
    std::default_random_engine random_generator;
    std::normal_distribution<float> normal_distribution;

    string name;
    scalar_statistic prediction_time, correction_time;
    int step, num_R_resets;
    float accel_noise_mag{}, gyro_noise_mag{};
    Matrix3f IMU_orientation;
    bool perfect_accel_prediction_;

    LocalizationTester(const string &testname) :
        settings("settings/localize_test_settings.xml"),
        sensor_data(make_unique<SensorData>()),
        sensors(make_unique<SensorBoss>(settings.f("SensorBoss", "accel_stddev"),
                                        settings.f("SensorBoss", "gyro_stddev"))),
        EKF(make_unique<StateEstimator>(settings.f("General", "main_timestep"),
                                        settings.f("State_Estimation", "pos_stddev"),
                                        settings.f("State_Estimation", "axis_stddev"),
                                        settings.f("State_Estimation", "vel_stddev"),
                                        settings.f("State_Estimation", "angvel_stddev"))),
        true_state(make_unique<RobotState>()), state(EKF->state), state_pred(EKF->state_pred),
        logger(),
        random_generator(0), normal_distribution(0, 1), name(testname),
        prediction_time(), correction_time(), step(0), num_R_resets(0), perfect_accel_prediction_(false)
        {
        EKF->set_damping_deceleration(settings.f("State_Estimation", "damping_deceleration"));
        perfect_accel_prediction_ = settings.b("perfect_accel_prediction");
        accel_noise_mag = settings.f(name.c_str(), "accel_noise");
        gyro_noise_mag = settings.f(name.c_str(), "gyro_noise");
        if (settings.b("File_Logging")) {
            logger = make_unique<ConvenientLogger>("data/localization_test_" + testname + ".log");
        }
        Eigen::Map<SensorVector>(vect_start(sensor_data.get())) = SensorVector::Zero();
        std::vector<float> IMU_orientation_raw({1., 0., 0.,
                                                0., 0., 1.,
                                                0., -1., 0.});
        IMU_orientation = Eigen::Map<Matrix3f>(IMU_orientation_raw.data());
        sensors->set_IMU_orientation(IMU_orientation_raw);
    }

    void do_state_estimation() {
        ++step;
        if (perfect_accel_prediction_)
            state_pred.acceleration = true_state->acceleration;
        chrono::time_point<chrono::steady_clock> timestart = chrono::steady_clock::now();
        EKF->predict();
        sensors->predict(state_pred, EKF->dt);
        chrono::time_point<chrono::steady_clock> timebetween = chrono::steady_clock::now();
        prediction_time.update(chrono::nanoseconds(timebetween - timestart).count());
        EKF->correct(*sensors);
        correction_time.update(chrono::nanoseconds(chrono::steady_clock::now() - timebetween).count());
        if (state.cached_rotation_count == 0)
            num_R_resets++;
    }

    void log_step(float timestamp) {
        if (logger) {
            logger->log("timestamp", timestamp);
            logger->obj_log("state", state);
            logger->obj_log("state_pred", state_pred);
            logger->obj_log("state_true", *true_state);
            logger->obj_log("sensors ", sensors->data);
            logger->obj_log("sensors_pred ", sensors->data_pred);
            logger->log("R", state.R);
            logger->log("R_pred", state_pred.R);
            logger->log("R_true", true_state->R);
            logger->log("State Covariance", EKF->state_covariance);
            logger->print();
        }
    }

    void print_results() {
        cout<< step <<" steps, " << num_R_resets << " R resets" << endl
            << "Prediction time: "<< prediction_time.mean << " ns, std_dev: " << prediction_time.std_dev
            << "    Correction time: " << correction_time.mean <<" ns, std_dev: " << correction_time.std_dev << endl
            << "Final state: "<< state.vect.transpose()<< endl
            << state.R << endl << endl;
    }

    void set_sensors_with_noise(const Vector3f &accel_clean, const Vector3f &gyro_clean, float timestamp) {
        Eigen::Map<Vector3f>(sensor_data->accel) = accel_clean;
        Eigen::Map<Vector3f>(sensor_data->gyro) = gyro_clean;
        for (int i = 0; i < 3; ++i) {
            sensor_data->accel[i] += accel_noise_mag / sqrtf(3) * normal_distribution(random_generator);
            sensor_data->gyro[i] += gyro_noise_mag / sqrtf(3) * normal_distribution(random_generator);
        }
        sensor_data->timestamp_us = timestamp * 1000000;
        sensors->update_sensors(sensor_data.get());
    }

    void set_sensors_from_world_frame(const Vector3f &accel_world, const Vector3f &gyro_world,
                                      const Matrix3f &true_orientation, float timestamp) {
        Vector3f true_accel = true_orientation.transpose() * IMU_orientation * (GRAVITY_ACCEL + accel_world);
        Vector3f true_gyro = true_orientation.transpose() * IMU_orientation * gyro_world;
        set_sensors_with_noise(true_accel, true_gyro, timestamp);
    }
};


class TestLocalization_Stationary : public LocalizationTester {
public:
    TestLocalization_Stationary() : LocalizationTester("stationary") {};
    int run() {
        if (settings.b(name.c_str(), "skip")) return -1;
        float duration = settings.f("stationary", "duration");

        cout<<"Simulating staying still for "<<duration<<" s. Start at "<<state.vect.transpose()<<endl;
        cout<<"Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        int num_steps = static_cast<int>(duration / EKF->dt);
        for (int i = 0; i < num_steps; i++) {
            set_sensors_from_world_frame(Vector3f::Zero(), Vector3f::Zero(), DEFAULT_ROTATION, i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        print_results();
        return 0;
    }
};

/* Robot starts moving. ramp up velocity for a time, then hold that velocity. */
class TestLocalization_StartMoving : public LocalizationTester {
public:
    TestLocalization_StartMoving() : LocalizationTester("start_moving") {};
    int run() {
        if (settings.b(name.c_str(), "skip")) return -1;
        float ramp_duration = settings.f("start_moving", "ramp_duration");
        float hold_duration = settings.f("start_moving", "hold_duration");
        float target_velocity = settings.f("start_moving", "target_velocity");

        Vector3f direction = Eigen::Map<Vector3f>(settings.vf("start_moving", "direction").data());
        direction /= direction.norm();
        Vector3f moving_accel = target_velocity / ramp_duration * direction;

        cout<< "Simulating Starting movement. Ramp to velocity of " << target_velocity << " m/s over "
            << ramp_duration << " s, then hold for " << hold_duration <<" s. Direction: " << direction.transpose()
            << endl << "Start at "<<state.vect.transpose()<<endl;
        cout<< "Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        int num_ramp_steps = static_cast<int>(ramp_duration / EKF->dt);
        int num_hold_steps = static_cast<int>(hold_duration / EKF->dt);
        for (int i = 0; i < num_ramp_steps; i++) {
            set_sensors_from_world_frame(moving_accel, Vector3f::Zero(), DEFAULT_ROTATION, i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        for (int i = num_ramp_steps; i < num_ramp_steps + num_hold_steps; i++) {
            set_sensors_from_world_frame(Vector3f::Zero(), Vector3f::Zero(), DEFAULT_ROTATION, i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        print_results();
        Vector3f true_pos = target_velocity * (ramp_duration / 2. + hold_duration) * direction;
        cout<< "True position: " << true_pos.transpose() << endl << endl;
        return 0;
    }
};


/* Robot starts moving. ramp up velocity for a time, then hold that velocity. */
class TestLocalization_Tilt : public LocalizationTester {
public:
    TestLocalization_Tilt() : LocalizationTester("tilt") {};
    int run() {
        if (settings.b(name.c_str(), "skip")) return -1;
        float move_duration = settings.f(name.c_str(), "move_duration");
        float hold_duration = settings.f(name.c_str(), "hold_duration");

        Vector3f axis_change;
        axis_change = Eigen::Map<Vector3f>(settings.vf(name.c_str(), "axis_change").data());

        cout << "Simulating tilting. Tilt around axis [" << axis_change.transpose() << "] over "
            << move_duration << " s, then hold for " << hold_duration << " s."
            << endl << "Start at "<<state.vect.transpose() << endl;
        cout<< "Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        const Vector3f target_axis = (state.axis() + axis_change);
        const int num_move_steps = static_cast<int>(move_duration / EKF->dt);
        const int num_hold_steps = static_cast<int>(hold_duration / EKF->dt);
        const Eigen::AngleAxisf angleaxis_per_tick(axis_change.norm() / num_move_steps,
                                                   axis_change / axis_change.norm());
        true_state->angvel() = axis_change / move_duration;
        cout << "Gyro (world frame): " << true_state->angvel().transpose() << endl;
//        cout << "Per tick axis: " << angleaxis_per_tick.axis().transpose()
//             << " angle: " << angleaxis_per_tick.angle() << endl;

        for (int i = 0; i < num_move_steps; i++) {
            true_state->axis() += true_state->angvel() * EKF->dt;
            true_state->calculate();
            set_sensors_from_world_frame(Vector3f::Zero(), true_state->angvel(), true_state->R, i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        cout << "Done moving, holding.\nIntermediate state: " << state.vect.transpose() << endl;
        true_state->angvel().setZero();
        for (int i = num_move_steps; i < num_move_steps + num_hold_steps; i++) {
            set_sensors_from_world_frame(Vector3f::Zero(), true_state->angvel(), true_state->R, i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        print_results();
        cout << "True axis, R: [" << true_state->axis().transpose() << "]\n" << true_state->R << "\n";
        cout << "True Axis to target axis error angle [rad]: "
             << acosf((float)(true_state->axis().transpose() * target_axis) / (true_state->axis().norm() * target_axis.norm())) << "\n";
        cout << "R angle error [rad]: "
             << Eigen::AngleAxisf(state.RT() * true_state->R).angle() << "\n";
        cout << "Axis angle error [rad]: "
             << acosf((float)(state.axis().transpose() * true_state->axis()) / (true_state->axis().norm() * state.axis().norm())) << "\n\n";
        return 0;
    }
};


/* Robot rocks back and forth as if on a swing. */
class TestLocalization_Swing : public LocalizationTester {
public:
    TestLocalization_Swing() : LocalizationTester("swing") {};
    int run() {
        if (settings.b(name.c_str(), "skip")) return -1;
        int num_swings = settings.f("swing", "num_swings");
        float period = settings.f("swing", "swing_period");
        float swing_height = settings.f("swing", "swing_height");
        float max_angle = settings.f("swing", "max_angle");

        Vector3f direction;
        direction << Eigen::Map<Vector3f>(settings.vf("swing", "direction").data());
        direction /= direction.norm();
        float swing_angular_freq = 2 * M_PI / period;
        Vector3f axis_direction = direction.cross(UP_DIR);
        state.vel() = swing_height * max_angle * swing_angular_freq * direction;
//        state.angvel() = DEFAULT_ROTATION.transpose() * axis_direction * max_angle * swing_angular_freq;

        log_step(-1.0);
        cout<< "Simulating swinging back and forth " << num_swings <<" times, " << period <<" s each." << endl
        << "Swing is suspended from a height of " << swing_height << " [m]. Maximum angle is " << max_angle
        << " [rad] Direction is " << direction.transpose() << endl << "Start at " << state.vect.transpose() << endl;
        cout<< "Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        int num_steps = static_cast<int>(num_swings * period / EKF->dt);
        for (int i = 0; i < num_steps; i++) {
            float t = i * EKF->dt;
            float swing_angle = max_angle * sin(swing_angular_freq * t);
            float swing_angle_rate = max_angle * swing_angular_freq * cos(swing_angular_freq * t);
            // actually "specific force", force per mass in world frame
            Vector3f swing_cord_force = Eigen::AngleAxisf(swing_angle, axis_direction)
                                        * (swing_angle_rate * swing_angle_rate * swing_height * UP_DIR);

            // TODO: replace DEFAULT_ROTATION with simulated rotation
            set_sensors_from_world_frame(swing_cord_force, axis_direction * swing_angle_rate, DEFAULT_ROTATION, t);
            do_state_estimation();
            log_step(t);
        }
        print_results();
        return 0;
    }
};

class TestLocalization_Orbit : public LocalizationTester {
public:
    TestLocalization_Orbit() : LocalizationTester("orbit") {};
    int run() {
        if (settings.b(name.c_str(), "skip")) return -1;
        float path_rad = settings.f("orbit", "path_rad");
        float path_freq = settings.f("orbit", "path_freq");
        int revolutions = settings.f("orbit", "num_orbits");

        float timestep = EKF->dt;
        cout << "Simulating circular path of radius " << 100 * path_rad << " cm" << endl
             << revolutions << " revolutions at " << path_freq << " rev/s" << endl
             << "Start at " << state.vect.transpose() << endl;

        float angvel_mag = path_freq * 2 * M_PI;
        float accel_mag = pow(angvel_mag, 2) * path_rad;
        // start moving in +x direction in a circle from y = 0 to y = 2*path_rad
        Vector3f accel_sensor_world = GRAVITY_ACCEL;
        Vector3f gyro_sensor_true = Vector3f::Zero();
        gyro_sensor_true(2) = angvel_mag;
        EKF->state.vel() = Vector3f(angvel_mag * path_rad, 0.0, 0.0);

        int num_steps = revolutions / path_freq / timestep;
        float angle = 0;

        for (int i = 0; i < num_steps; i++) {
            accel_sensor_world(FORWARD_IDX) = -accel_mag * sin(angle);
            accel_sensor_world(LEFT_IDX) = accel_mag * cos(angle);

            // TODO: replace DEFAULT_ROTATION with simulated rotation
            set_sensors_from_world_frame(accel_sensor_world, gyro_sensor_true, DEFAULT_ROTATION, i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
            angle += angvel_mag * timestep;
        }
        print_results();
        return 0;
    }
};

int main() {
    unique_ptr<TestLocalization_Stationary> test_stationary = make_unique<TestLocalization_Stationary>();
    test_stationary->run();

    unique_ptr<TestLocalization_StartMoving> test_startmoving = make_unique<TestLocalization_StartMoving>();
    test_startmoving->run();

    unique_ptr<TestLocalization_Tilt> test_tilt = make_unique<TestLocalization_Tilt>();
    test_tilt->run();

    unique_ptr<TestLocalization_Swing> test_swing = make_unique<TestLocalization_Swing>();
    test_swing->run();

    unique_ptr<TestLocalization_Orbit> test_orbit = make_unique<TestLocalization_Orbit>();
    test_orbit->run();
    cout<<"Done Testing Localization"<<endl;
    return 0;
}

