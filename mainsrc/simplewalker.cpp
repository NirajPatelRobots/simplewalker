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
#define MSG_WAIT_TIME_US 1000 //wait this long before skipipng



int main() {
    WalkerSettings settings("settings/settings.xml"); //settings

    ControlStateMsg *controlstate = new ControlStateMsg;
    controlstate->ID = 0;
    RobotStateMsg *sendstate = new RobotStateMsg;
    sendstate->ID = 0;
    sendstate->errcode = 0;
    Communicator comm(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg));
    comm.start_server(settings.f("General", "state_send_port"), RobotStateMsgID,
                        sizeof(RobotStateMsg), (const char *)sendstate);
    SensorBoss *sensors = new SensorBoss(controlstate->accel,
                        settings.f("SensorBoss", "accel_stddev"), settings.f("SensorBoss", "gyro_stddev"));
    float timestep = settings.f("General", "main_timestep");
    StateEstimator *EKF = new StateEstimator(timestep, *sensors, settings.f("State_Estimation", "pos_stddev"), 
                                            settings.f("State_Estimation", "euler_stddev"),
                                            settings.f("State_Estimation", "vel_stddev"),
                                            settings.f("State_Estimation", "angvel_stddev"));

    chrono::milliseconds looptime(static_cast<int>(1000 * timestep));
    chrono::microseconds msgwaittime(MSG_WAIT_TIME_US);
    chrono::time_point<chrono::steady_clock> latereadtime;
    bool ERR_msg_late = false;
    int num_msgs = 0;
    Logger logger(settings.b("Logger", "newline"));
    //Logger savelog("data/statelog.csv"); TODO BUG
    Logtimes logtimes;

    //std::this_thread::sleep_until(chrono::steady_clock::now() + looptime); //give it time to receive
    std::cout<<"Start main loop, T = "<< looptime.count() << " ms" << std::endl;
    auto timestart = chrono::steady_clock::now();
    chrono::time_point<chrono::steady_clock> loopstart = timestart;
    while (true) {
        set_logtime(logtimes.sleep);
        comm.handle_messages();
        while ((num_msgs = comm.read_message((char *)controlstate)) < 0 ) {
            comm.handle_messages();
            latereadtime = chrono::steady_clock::now();
            ERR_msg_late = true;
            if (latereadtime - loopstart > msgwaittime) {
                std::cout<<"ERROR: message missing -";
                break;
            }
        }
        if (ERR_msg_late) {
            std::cout<<"late  \n";
            loopstart = latereadtime; // slow down time so we don't get ahead
            ERR_msg_late = false;
        }
        //sendstate->timestamp_us = chrono::microseconds(loopstart - timestart).count(); //BUG
        for (int i = 0; i < N; ++i) {
            float *float_out = sendstate->pos + i;
            *float_out = EKF->state.vect[i];
        }
        if (settings.b("General", "state_send")) comm.broadcast_message(settings.f("General", "broadcast_rate_div"));
        set_logtime(logtimes.commreceive);

        EKF->predict();
        set_logtime(logtimes.predict);
        EKF->correct();
        set_logtime(logtimes.correct);
        if (settings.b("Logger", "log_times")) logger.log(logtimes);
        if (settings.b("Logger", "log_sensor")) logger.log(*sensors);
        if (settings.b("Logger", "log_state")) logger.log(EKF->state);
        //savelog.log(EKF->state);
        if (logger.print(settings.f("Logger", "skip_every"))) {
            set_logtime(logtimes.log);
        }

        loopstart += looptime;
        std::this_thread::sleep_until(loopstart);
    }
}
