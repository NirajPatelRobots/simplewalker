/* Simplewalker main program

TODO:
    BUG: Sometimes, correlated with receiving message late first, state estimation takes 30 ms. 
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
    WalkerSettings settings("settings/settings.xml"); //settings

    ControlStateMsg *controlstate = new ControlStateMsg;
    controlstate->ID = 0;
    Communicator comm(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg));
    SensorBoss *sensors = new SensorBoss(controlstate->accel,
                        settings.f("SensorBoss", "accel_stddev"), settings.f("SensorBoss", "gyro_stddev"));
    float timestep = settings.f("General", "main_timestep");
    StateEstimator *EKF = new StateEstimator(timestep, *sensors, settings.f("State_Estimation", "pos_stddev"), 
                                            settings.f("State_Estimation", "euler_stddev"),
                                            settings.f("State_Estimation", "vel_stddev"),
                                            settings.f("State_Estimation", "angvel_stddev"));

    chrono::milliseconds looptime(LOOP_TIME_MS);
    chrono::microseconds msgwaittime(MSG_WAIT_TIME_US);
    chrono::time_point<chrono::steady_clock> latereadtime;
    bool ERR_msg_late = false;
    int num_msgs = 0;
    Logger logger(settings.b("Logger", "newline"));
    //Logger savelog("data/statelog.csv"); TODO BUG
    Logtimes logtimes;

    std::cout<<"Start main loop"<<std::endl;
    auto loopstart = chrono::steady_clock::now();
    while (true) {
        set_logtime(logtimes.sleep);
        comm.handle_messages();
        while ((num_msgs = comm.read_message((char *)controlstate)) < 0 && latereadtime - loopstart < msgwaittime) {
            comm.handle_messages();
            latereadtime = chrono::steady_clock::now();
        }
        ERR_msg_late = (latereadtime > loopstart);
        if (ERR_msg_late) {
            std::cout<<"late  \n";
            loopstart = latereadtime; // slow down time so we don't get ahead
        }
        set_logtime(logtimes.commreceive);

        EKF->predict();
        set_logtime(logtimes.predict);
        EKF->correct();
        set_logtime(logtimes.correct);
        logger.log("SensorBoss", "accel_stddev", settings);
        logger.log("SensorBoss", "gyro_stddev", settings);
        if (settings.b("Logger", "log_times")) logger.log(logtimes);
        if (settings.b("Logger", "log_sensor")) logger.log(*sensors);
        if (settings.b("Logger", "log_state")) logger.log(EKF->state);
        //savelog.log(EKF->state);
        if (logger.print(10)) {
            set_logtime(logtimes.log);
        }

        loopstart += looptime;
        std::this_thread::sleep_until(loopstart);
    }
}
