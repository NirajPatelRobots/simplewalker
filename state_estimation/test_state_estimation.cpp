/* Code for testing state estimator
TODO:
    read in parameters instead of hardcoded
    sensor noise
March 2022 */
#include "state_estimation.hpp"
#include <cmath>
#include <time.h>
#include <iostream>
#include <fstream>

int main() {
    // EKF parameters
    float accel_stddev = 1.0, gyro_stddev = 0.01, timestep = 0.03;
    float pos_stddev = 0.01, euler_stddev = 0.05, vel_stddev = 1.0, angvel_stddev = 1.0;
    //path parameters
    float path_rad = 0.03; //[m], circular path radius
    float path_freq = 1.0; //[Hz], path revolutions per second
    float wobble_mag = 0.3; //[rad], magnitude of alpha-beta wobble
    float wobble_freq_scale = 3.0; // frequence of wobble compared to path_freq
    int revolutions = 3;
    std::cout<<"Simulating circular path of radius "<<100*path_rad<<" cm\n";
    std::cout<<revolutions<<" revolutions at "<<path_freq<<" rev/s\n";
    std::cout<<"A "<<wobble_mag<<" rad wobble between alpha and beta, "<<wobble_freq_scale<<" per revolution\n";

    StateEstimator EKF(accel_stddev, gyro_stddev, timestep, 
                        pos_stddev, euler_stddev, vel_stddev, angvel_stddev);
    float angvel_mag = path_freq * 2 * M_PI;
    float accel_mag = pow(angvel_mag, 2) * path_rad;
    // start moving in +x direction in a circle at y = -path_rad
    Vector3f gyro_data = {0.0, 0.0, angvel_mag};
    Vector3f accel_data = Vector3f::Zero();
    EKF.state.vel() = Vector3f(angvel_mag * path_rad, 0.0, 0.0);
    
    int num_steps = revolutions / path_freq / timestep;
    float angle = 0, avg_predict_time = 0, avg_correct_time = 0;
    long int start_time, predict_time, correct_time;
    const long int ONE_SECOND = 1000000000;
	struct timespec gettime_now, waittime = {0, 0};
    std::ofstream logfile("localizeTestLog.csv", std::ios::out | std::ios::trunc);
    logfile << estimator_CSV_header() << 0.0 << "," << EKF;

    for (int i =0; i < num_steps; i++) {
        accel_data(0) = -accel_mag * sin(angle);
        accel_data(1) = -accel_mag * cos(angle);
        gyro_data(0) = -wobble_mag * cos(wobble_freq_scale * angle);
        gyro_data(1) = -wobble_mag * sin(wobble_freq_scale * angle);
	    clock_gettime(CLOCK_REALTIME, &gettime_now);
	    start_time = gettime_now.tv_nsec;

        EKF.predict();
        clock_gettime(CLOCK_REALTIME, &gettime_now);
		predict_time = gettime_now.tv_nsec - start_time;
		if (predict_time < 0) predict_time += ONE_SECOND;

        EKF.correct(accel_data, gyro_data);
        clock_gettime(CLOCK_REALTIME, &gettime_now);
		correct_time = gettime_now.tv_nsec - start_time;
		if (correct_time < 0) correct_time += ONE_SECOND;

        if (logfile.is_open()) {
            logfile << i * timestep << "," << EKF; 
        }
        avg_predict_time += (float)predict_time / num_steps;
        avg_correct_time += (float)correct_time / num_steps;
        waittime.tv_nsec = (long int)(timestep * ONE_SECOND) - predict_time - correct_time;
        angle += angvel_mag * timestep;
        nanosleep(&waittime, NULL);
    }
    logfile.close();
    std::cout<<EKF.state;
    std::cout<<"Prediction time: " << 1e-6 * avg_predict_time
            << " ms, Correction time: " << 1e-6 * avg_correct_time << " ms\n";
}

