/* Simplewalker main program

*/
#include "maincomp_comm.hpp"
#include "state_estimation.hpp"
#include "logger.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace chrono = std::chrono;
#define LOOP_TIME_MS 30
#define MSG_WAIT_TIME_US 1000

int main() {
    ControlStateMsg *controlstate = new ControlStateMsg;
    controlstate->ID = 0;
    Communicator comm(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg));
    float accel_stddev = 1.0, gyro_stddev = 0.01;
    SensorBoss *sensors = new SensorBoss(controlstate->accel, accel_stddev, gyro_stddev);
    // EKF parameters
    float timestep = 0.03, pos_stddev = 0.01, euler_stddev = 0.05, vel_stddev = 1.0, angvel_stddev = 1.0;
    StateEstimator *EKF = new StateEstimator(timestep, *sensors,
                                    pos_stddev, euler_stddev, vel_stddev, angvel_stddev);

    chrono::milliseconds looptime(LOOP_TIME_MS);
    chrono::microseconds msgwaittime(MSG_WAIT_TIME_US);
    chrono::time_point<chrono::steady_clock> readtime;
    bool ERR_msg_late = false;
    int num_msgs = 0;
    Logger logger;
    Logtimes logtimes;

    auto loopstart = chrono::steady_clock::now();
    while (true) {
        start_logtiming(loopstart);
        comm.handle_messages();
        while ((num_msgs = comm.read_message((char *)controlstate)) < 0 && readtime - loopstart < msgwaittime) {
            comm.handle_messages();
            readtime = chrono::steady_clock::now();
        }
        ERR_msg_late = (readtime > loopstart);
        if (ERR_msg_late) {
            std::cout<<"late  \n";
            loopstart = readtime; // slow down time so we don't get ahead
        }
        set_logtime(logtimes.commreceive);

        EKF->predict();
        set_logtime(logtimes.predict);
        EKF->correct();
        set_logtime(logtimes.correct);
        logger.log(logtimes);
        logger.log(*sensors);
        if (logger.print(10)) {
            set_logtime(logtimes.log);
        }

        loopstart += looptime;
        std::this_thread::sleep_until(loopstart);
    }
}