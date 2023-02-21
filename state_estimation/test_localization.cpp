/* Code for testing state estimator
TODO:
    true_state in LocalizationTester
    move parts of tests into parent LocalizationTester
March 2022 */
#include "state_estimation.hpp"
#include "convenientLogger.hpp"
#include "messages.h"
#include <chrono>
#include <fstream>
#include <random>
namespace chrono = std::chrono;

class LocalizationTester {
public:
    const WalkerSettings walkerSettings;
    unique_ptr<SensorData> sensor_data;
    unique_ptr<SensorBoss> sensors;
    unique_ptr<StateEstimator> EKF;
    RobotState &state;
    RobotState &state_pred;
    unique_ptr<ConvenientLogger> logger;
    std::default_random_engine random_generator;
    std::normal_distribution<float> normal_distribution;

    string name;
    Eigen::Map<Vector3f> accel_sensor, gyro_sensor;
    scalar_statistic prediction_time, correction_time;
    int step;
    float accel_noise_mag, gyro_noise_mag;

    LocalizationTester(const string &testname) :
        walkerSettings("settings/settings.xml"),
        sensor_data(make_unique<SensorData>()),
        sensors(make_unique<SensorBoss>(walkerSettings.f("SensorBoss", "accel_stddev"),
                                        walkerSettings.f("SensorBoss", "gyro_stddev"))),
        EKF(make_unique<StateEstimator>(walkerSettings.f("General", "main_timestep"),
                                        walkerSettings.f("State_Estimation", "pos_stddev"), 
                                        walkerSettings.f("State_Estimation", "axis_stddev"),
                                        walkerSettings.f("State_Estimation", "vel_stddev"),
                                        walkerSettings.f("State_Estimation", "angvel_stddev"))),
        state(EKF->state), state_pred(EKF->state_pred),
        logger(make_unique<ConvenientLogger>("data/localization_test_" + testname + ".log")),
        random_generator(0), normal_distribution(0, 1),
        name(testname), accel_sensor(sensor_data->accel), gyro_sensor(sensor_data->gyro),
        prediction_time(), correction_time(), step(0) {
        for (unsigned i = 0; i < sizeof(SensorData); ++i) {
            ((char *)sensor_data.get())[i] = 0;  // TODO ugly
        }
    }

    void do_state_estimation() {
        ++step;
        chrono::time_point<chrono::steady_clock> timestart = chrono::steady_clock::now();
        EKF->predict();
        sensors->predict(state_pred, state, EKF->dt);
        chrono::time_point<chrono::steady_clock> timebetween = chrono::steady_clock::now();
        prediction_time.update(chrono::nanoseconds(timebetween - timestart).count());
        EKF->correct(*sensors);
        correction_time.update(chrono::nanoseconds(chrono::steady_clock::now() - timebetween).count());
    }

    void log_step(float timestamp) {
        logger->log("timestamp", timestamp);
        logger->obj_log("state", state);
        logger->obj_log("state_pred", state_pred);
        logger->log("sensors ", sensors->data);
        logger->log("sensors_pred ", sensors->data_pred);
        logger->log("R", state.R);
        logger->log("R_pred", state_pred.R);
        logger->print();
    }

    void print_results() {
        cout<< step <<" steps, prediction time: "<< prediction_time.mean << " ns, std_dev: " << prediction_time.std_dev
            << endl << "correction time: " << correction_time.mean <<" ns, std_dev: " << correction_time.std_dev << endl
            << "Final state: "<< state.vect.transpose()<< endl
            << state.R << endl << endl;
    }

    void set_sensors_with_noise(const Vector3f &accel_clean, const Vector3f &gyro_clean, float timestamp) {
        accel_sensor = accel_clean;
        gyro_sensor = gyro_clean;
        for (int i = 0; i < 3; ++i) {
            accel_sensor(i) += accel_noise_mag / sqrtf(3) * normal_distribution(random_generator);
            gyro_sensor(i) += gyro_noise_mag / sqrtf(3) * normal_distribution(random_generator);
        }
        sensor_data->timestamp_us = timestamp * 1e6;
        sensors->update_sensors(sensor_data.get());
    }
};


class TestLocalization_Stationary : public LocalizationTester {
public:
    TestLocalization_Stationary() : LocalizationTester("stationary") {};
    int run(const WalkerSettings &testSettings) {
        if (testSettings.b("stationary", "skip")) return -1;
        float duration = testSettings.f("stationary", "duration");

        accel_noise_mag = testSettings.f("stationary", "accel_noise");
        gyro_noise_mag = testSettings.f("stationary", "gyro_noise");
        Vector3f accel_sensor_true = DEFAULT_ROTATION.transpose() * IMU_GRAVITY;
        log_step(-1.0);
        cout<<"Simulating staying still for "<<duration<<" s. Start at "<<state.vect.transpose()<<endl;
        cout<<"Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        int num_steps = static_cast<int>(duration / EKF->dt);
        for (int i = 0; i < num_steps; i++) {
            set_sensors_with_noise(accel_sensor_true, Vector3f::Zero(), i * EKF->dt);
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
    int run(const WalkerSettings &testSettings) {
        if (testSettings.b("start_moving", "skip")) return -1;
        float ramp_duration = testSettings.f("start_moving", "ramp_duration");
        float hold_duration = testSettings.f("start_moving", "hold_duration");
        float target_velocity = testSettings.f("start_moving", "target_velocity");

        accel_noise_mag = testSettings.f("start_moving", "accel_noise");
        gyro_noise_mag = testSettings.f("start_moving", "gyro_noise");
        Vector3f direction;
        direction << Eigen::Map<Vector3f>(testSettings.vf("start_moving", "direction").data());
        direction /= direction.norm();
        Vector3f accel_stationary = DEFAULT_ROTATION.transpose() * IMU_GRAVITY;
        Vector3f accel_sensor_true = accel_stationary + target_velocity / ramp_duration * direction;

        log_step(-1.0);
        cout<< "Simulating Starting movement. Ramp to velocity of " << target_velocity << " m/s over "
            << ramp_duration << " s, then hold for " << hold_duration <<" s. Start at "<<state.vect.transpose()<<endl;
        cout<< "Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        int num_ramp_steps = static_cast<int>(ramp_duration / EKF->dt);
        int num_hold_steps = static_cast<int>(hold_duration / EKF->dt);
        for (int i = 0; i < num_ramp_steps; i++) {
            set_sensors_with_noise(accel_sensor_true, Vector3f::Zero(), i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        accel_sensor_true = accel_stationary;
        for (int i = num_ramp_steps; i < num_ramp_steps + num_hold_steps; i++) {
            set_sensors_with_noise(accel_sensor_true, Vector3f::Zero(), i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
        }
        print_results();
        Vector3f true_pos = target_velocity * (ramp_duration / 2. + hold_duration) * direction;
        cout<< "True position: " << true_pos.transpose() << endl << endl;
        return 0;
    }
};

/* Robot rocks back and forth as if on a swing. */
class TestLocalization_Swing : public LocalizationTester {
public:
    TestLocalization_Swing() : LocalizationTester("swing") {};
    int run(const WalkerSettings &testSettings) {
        if (testSettings.b("swing", "skip")) return -1;
        int num_swings = testSettings.f("swing", "num_swings");
        float period = testSettings.f("swing", "swing_period");
        float swing_height = testSettings.f("swing", "swing_height");
        float max_angle = testSettings.f("swing", "max_angle");

        accel_noise_mag = testSettings.f("swing", "accel_noise");
        gyro_noise_mag = testSettings.f("swing", "gyro_noise");
        Vector3f direction;
        direction << Eigen::Map<Vector3f>(testSettings.vf("swing", "direction").data());
        direction /= direction.norm();
        float swing_angular_freq = 2 * M_PI / period;
        state.vel() = swing_height * max_angle * swing_angular_freq * direction;

        log_step(-1.0);
        cout<< "Simulating swinging back and forth " << num_swings <<" times, " << period <<" s each." << endl
        << "Swing is suspended from a height of " << swing_height << " [m]. Maximum angle is " << max_angle
        << " [rad]" << endl << "Start at " << state.vect.transpose() << endl;
        cout<< "Accel noise: " << accel_noise_mag << " [m/s^2] gyro noise: " << gyro_noise_mag << " [rad/s]" << endl;

        Vector3f axis_direction = direction.cross(UP_DIR);
        int num_steps = static_cast<int>(num_swings * period / EKF->dt);
        for (int i = 0; i < num_steps; i++) {
            float t = i * EKF->dt;
            float swing_angle = max_angle * sin(swing_angular_freq * t);
            float swing_angle_rate = max_angle * swing_angular_freq * cos(swing_angular_freq * t);
            Vector3f gyro_sensor_true =  DEFAULT_ROTATION.transpose() * axis_direction * swing_angle_rate;
            // these forces are actually "specific forces", force per mass
            Vector3f swing_cord_force = swing_angle_rate * swing_angle_rate * swing_height * UP_DIR;
            Vector3f force_gravity = Eigen::AngleAxisf(-swing_angle, axis_direction) * IMU_GRAVITY; //UP_DIR * -9.81;
            Vector3f accel_sensor_true =  DEFAULT_ROTATION.transpose() * (swing_cord_force + force_gravity);

            set_sensors_with_noise(accel_sensor_true, gyro_sensor_true, i * EKF->dt);
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
    int run(const WalkerSettings &testSettings) {
        if (testSettings.b("orbit", "skip")) return -1;
        float path_rad = testSettings.f("orbit", "path_rad");
        float path_freq = testSettings.f("orbit", "path_freq");
        int revolutions = testSettings.f("orbit", "num_orbits");
        accel_noise_mag = testSettings.f("orbit", "accel_noise");
        gyro_noise_mag = testSettings.f("orbit", "gyro_noise");

        float timestep = EKF->dt;
        cout << "Simulating circular path of radius " << 100 * path_rad << " cm\n";
        cout << revolutions << " revolutions at " << path_freq << " rev/s\n";

        float angvel_mag = path_freq * 2 * M_PI;
        float accel_mag = pow(angvel_mag, 2) * path_rad;
        // start moving in +x direction in a circle at y = -path_rad
        Vector3f accel_sensor_true = DEFAULT_ROTATION.transpose() * IMU_GRAVITY;
        Vector3f gyro_sensor_true = Vector3f::Zero();
        gyro_sensor_true(2) = angvel_mag;
        EKF->state.vel() = Vector3f(angvel_mag * path_rad, 0.0, 0.0);

        int num_steps = revolutions / path_freq / timestep;
        float angle = 0;

        for (int i = 0; i < num_steps; i++) {
            accel_sensor_true(0) = -accel_mag * sin(angle);
            accel_sensor_true(1) = -accel_mag * cos(angle);

            set_sensors_with_noise(accel_sensor_true, Vector3f::Zero(), i * EKF->dt);
            do_state_estimation();
            log_step(i * EKF->dt);
            angle += angvel_mag * timestep;
        }
        print_results();
        return 0;
    }
};

int main() {
    const WalkerSettings testSettings("settings/localize_test_settings.xml");
    unique_ptr<TestLocalization_Stationary> test_stationary = make_unique<TestLocalization_Stationary>();
    test_stationary->run(testSettings);

    unique_ptr<TestLocalization_StartMoving> test_startmoving = make_unique<TestLocalization_StartMoving>();
    test_startmoving->run(testSettings);

    unique_ptr<TestLocalization_Swing> test_swing = make_unique<TestLocalization_Swing>();
    test_swing->run(testSettings);

    unique_ptr<TestLocalization_Orbit> test_orbit = make_unique<TestLocalization_Orbit>();
    test_orbit->run(testSettings);
    cout<<"Done Testing Localization"<<endl;
    return 0;
}

