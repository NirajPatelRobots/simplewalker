/* Code for testing state estimator
TODO:
    read in test parameters
    sensor noise
    tests are classes inherited from LocalizationTester
March 2022 */
#include "state_estimation.hpp"
#include "convenientLogger.hpp"
#include "messages.h"
#include <chrono>
#include <fstream>
namespace chrono = std::chrono;

class LocalizationTester {
public:
    const WalkerSettings walkerSettings;
    shared_ptr<ControlStateMsg> controlstate;
    shared_ptr<SensorBoss> sensors;
    shared_ptr<StateEstimator> EKF;
    RobotState &state;
    RobotState &state_pred;
    ConvenientLogger logger;

    string name;
    Eigen::Map<Vector3f> accel_sensor, gyro_sensor;
    chrono::nanoseconds prediction_time, correction_time;
    int step;

    LocalizationTester(const string testname) :
        walkerSettings("settings/settings.xml"),
        controlstate(new ControlStateMsg),
        sensors(new SensorBoss(controlstate->accel,
                walkerSettings.f("SensorBoss", "accel_stddev"), walkerSettings.f("SensorBoss", "gyro_stddev"))),
        EKF(new StateEstimator(walkerSettings.f("General", "main_timestep"), *sensors, 
                               walkerSettings.f("State_Estimation", "pos_stddev"), 
                               walkerSettings.f("State_Estimation", "axis_stddev"),
                               walkerSettings.f("State_Estimation", "vel_stddev"),
                               walkerSettings.f("State_Estimation", "angvel_stddev"))),
        state(EKF->state), state_pred(EKF->state_pred),
        logger("data/localization_test_" + testname + ".log"),
        name(testname), accel_sensor(controlstate->accel), gyro_sensor(controlstate->gyro),
        prediction_time(0), correction_time(0), step(0) {
        for (unsigned i = 0; i < sizeof(ControlStateMsg); ++i) {
            ((char *)controlstate.get())[i] = 0;  // TODO ugly
        }
    }

    void do_state_estimation(void) {
        ++step;
        chrono::time_point<chrono::steady_clock> timestart = chrono::steady_clock::now();
        EKF->predict();
        chrono::time_point<chrono::steady_clock> timebetween = chrono::steady_clock::now();
        prediction_time = prediction_time + timebetween - timestart;
        EKF->correct();
        correction_time += chrono::steady_clock::now() - timebetween;
    }

    void log_step(float timestamp) {
        logger.log("timestamp", timestamp);
        logger.obj_log("state", state);
        logger.obj_log("state_pred", state_pred);
        logger.obj_log("sensors", *sensors);
        logger.log("R", state.R);
        logger.print();
    }

    void print_results(void) {
        cout<< step <<" steps, prediction time: "<< prediction_time.count() / step <<" ns, correction time: "
            << correction_time.count() / step <<" ns"<< endl << "Final state: "<< state.vect.transpose()<< endl
            << state.R << endl;
    }
};


int test_localization_stationary(float duration, float accel_noise, float gyro_noise) {
    /*duration [s] how long simulated stationary for
    accel_noise [m/s^2] std dev of accel noise
    gyro_noise [m/s^2] std dev of gyro noise*/
    unique_ptr<LocalizationTester> tester(new LocalizationTester("stationary"));
    RobotState &state = tester->state;
    float timestep = tester->EKF->dt;
    tester->accel_sensor = DEFAULT_ROTATION.transpose() * IMU_GRAVITY;
    cout<<"Simulating staying still. Start at "<<state.vect.transpose()<<endl<<state.R<<endl;

    int num_steps = static_cast<int>(duration / timestep);

    for (int i = 0; i < num_steps; i++) {
        // TODO noise
        tester->log_step(i * timestep);
        tester->do_state_estimation();
    }
    tester->print_results();
    return 0;
}

int test_localization_orbit(float path_rad, float path_freq, float wobble_mag, float wobble_freq_scale, int revolutions) {
    /*path_rad [m],    circular path radius
    path_freq [Hz],    path revolutions per second
    wobble_mag [rad],  magnitude of alpha-beta wobble
    wobble_freq_scale  frequence of wobble compared to path_freq 
    revolutions        number of times to orbit

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
    float path_rad = 0.03;
    float path_freq = 1.0;
    float wobble_mag = 0.0;
    float wobble_freq_scale = 3.0;
    int revolutions = 3;
    test_localization_stationary(5.0, 0.0, 0.0);
    test_localization_orbit(path_rad, path_freq, wobble_mag, wobble_freq_scale, revolutions);
    cout<<"Done Testing Localization"<<endl;
    return 0;
}

