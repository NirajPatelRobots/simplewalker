/* Code for testing state estimator
TODO:
    test_localization_orbit
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
    ConvenientLogger logger;
    std::default_random_engine random_generator;
    std::normal_distribution<float> normal_distribution;

    string name;
    Eigen::Map<Vector3f> accel_sensor, gyro_sensor;
    scalar_statistic prediction_time, correction_time;
    int step;
    float accel_noise_mag, gyro_noise_mag;

    LocalizationTester(const string testname) :
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
        logger("data/localization_test_" + testname + ".log"),
        random_generator(0), normal_distribution(0, 1),
        name(testname), accel_sensor(sensor_data->accel), gyro_sensor(sensor_data->gyro),
        prediction_time(), correction_time(), step(0) {
        for (unsigned i = 0; i < sizeof(ControlStateMsg); ++i) {
            ((char *)sensor_data.get())[i] = 0;  // TODO ugly
        }
    }

    void do_state_estimation(void) {
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
        logger.log("timestamp", timestamp);
        logger.obj_log("state", state);
        logger.obj_log("state_pred", state_pred);
        logger.log("sensors ", sensors->data);
        logger.log("sensors_pred ", sensors->data_pred);
        logger.log("R", state.R);
        logger.log("R_pred", state_pred.R);
        logger.print();
    }

    void print_results(void) {
        cout<< step <<" steps, prediction time: "<< prediction_time.mean << " ns, std_dev: " << prediction_time.std_dev
            << endl << "correction time: " << correction_time.mean <<" ns, std_dev: " << correction_time.std_dev << endl
            << "Final state: "<< state.vect.transpose()<< endl
            << state.R << endl; 
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
    TestLocalization_Stationary(void) : LocalizationTester("stationary") {};
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

int test_localization_orbit(const WalkerSettings &testSettings) {
    if (testSettings.b("orbit", "skip")) return -1;
    /*float path_rad = testSettings.f("orbit", "path_rad");
    float path_freq = testSettings.f("orbit", "path_freq");
    int revolutions = testSettings.i("orbit", "num_orbits")
    float wobble_mag = testSettings.f("orbit", "wobble_mag");

    unique_ptr<LocalizationTester> tester(new LocalizationTester("Orbit"));
    float timestep = tester->EKF->dt;
    cout<<"Simulating circular path of radius "<<100*path_rad<<" cm\n";
    cout<<revolutions<<" revolutions at "<<path_freq<<" rev/s\n";
    cout<<"A "<<wobble_mag<<" rad wobble between alpha and beta, "<<wobble_freq_scale<<" per revolution\n";

    float angvel_mag = path_freq * 2 * M_PI;
    float accel_mag = pow(angvel_mag, 2) * path_rad;
    // start moving in +x direction in a circle at y = -path_rad
    tester->gyro_sensor(2) = angvel_mag;
    EKF.state.vel() = Vector3f(angvel_mag * path_rad, 0.0, 0.0);
    
    int num_steps = revolutions / path_freq / timestep;
    float angle = 0;

    for (int i =0; i < num_steps; i++) {
        tester->accel_sensor(0) = -accel_mag * sin(angle);
        tester->accel_sensor(1) = -accel_mag * cos(angle);
        tester->gyro_sensor(0) = -wobble_mag * cos(wobble_freq_scale * angle);
        tester->gyro_sensor(1) = -wobble_mag * sin(wobble_freq_scale * angle);

        tester->log_step(i * timestep);
        tester->do_state_estimation();
        angle += angvel_mag * timestep;
    }
    tester->printresults(); */
    return 0;
}

int main() {
    const WalkerSettings testSettings("settings/localize_test_settings.xml");
    //test_localization_stationary(testSettings);
    TestLocalization_Stationary *test_stationary = new TestLocalization_Stationary();
    test_stationary->run(testSettings);
    delete test_stationary;

    test_localization_orbit(testSettings);
    cout<<"Done Testing Localization"<<endl;
    return 0;
}

