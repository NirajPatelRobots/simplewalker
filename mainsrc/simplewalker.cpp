/* Simplewalker main program

TODO:
    BUG: Sometimes, correlated with receiving message late first, state estimation takes 30 ms. 
*/
#include "maincomp_comm.hpp"
#include "state_estimation.hpp"
#include "convenientLogger.hpp"
#include <iostream>
#include <memory>
#include <chrono>
#include <thread>

namespace chrono = std::chrono;
const static int MSG_WAIT_TIME_US {1000}; //wait this long before skipipng

void set_state_msg(RobotStateMsg *msg, const RobotState &state, chrono::duration<float> timestamp) {
    msg->timestamp_us = chrono::duration_cast<chrono::microseconds>(timestamp).count();
    for (int i = 0; i < N; ++i) {
        float *float_out = msg->pos + i;
        *float_out = state.vect[i];
    }
}

int main() {
    WalkerSettings settings("settings/settings.xml"); //settings

    std::unique_ptr<ControlStateMsg> controlstate(new ControlStateMsg);
    controlstate->ID = 0;
    std::unique_ptr<RobotStateMsg> sendstate(new RobotStateMsg);
    sendstate->ID = RobotStateMsgID;
    sendstate->errcode = 0;
    for (int i = 0; i < 6; ++i) {sendstate->leg_pos[i] = 0; sendstate->leg_vel[i] = 0;}

    std::unique_ptr<Communicator> comm(new Communicator(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg)));
    comm->start_server(settings.f("General", "state_send_port"), RobotStateMsgID,
                        sizeof(RobotStateMsg), (const char *)sendstate.get());
    if (settings.b("General", "state_send")) comm->try_connect();
    
    std::unique_ptr<SensorBoss> sensors(new SensorBoss(controlstate->accel,
                        settings.f("SensorBoss", "accel_stddev"), settings.f("SensorBoss", "gyro_stddev")));
    float timestep = settings.f("General", "main_timestep");
    const bool prediction_only = settings.b("State_Estimation", "prediction_only");
    if (prediction_only) std::cout<<"\tPrediction Only, Ignoring Sensors"<<std::endl;
    std::unique_ptr<StateEstimator> EKF(new StateEstimator(timestep, *sensors, 
                                            settings.f("State_Estimation", "pos_stddev"), 
                                            settings.f("State_Estimation", "axis_stddev"),
                                            settings.f("State_Estimation", "vel_stddev"),
                                            settings.f("State_Estimation", "angvel_stddev")));
    RobotState &state = EKF->state;

    chrono::milliseconds looptime(static_cast<int>(1000 * timestep));
    chrono::microseconds msgwaittime(MSG_WAIT_TIME_US);
    chrono::time_point<chrono::steady_clock> latereadtime;
    bool ERR_msg_late = false;
    int num_msgs = 0;
    shared_ptr<ConvenientLogger> logger{std::static_pointer_cast<ConvenientLogger>(stdlogger)};
    //Logger savelog("data/statelog.csv"); TODO BUG
    Logtimes logtimes;

    state.angvel() << 0, 0, M_PI / 5.0;

    std::cout<<"\tStart main loop, T = "<< looptime.count() << " ms" << std::endl;
    chrono::time_point<chrono::steady_clock> timestart = chrono::steady_clock::now();
    chrono::time_point<chrono::steady_clock> loopstart = timestart;
    while (true) {
        set_logtime(logtimes.sleep);
        comm->handle_messages();
        while ((num_msgs = comm->read_message((char *)controlstate.get())) < 0 ) {
            comm->handle_messages();
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
        set_logtime(logtimes.commreceive);
        set_state_msg(sendstate.get(), EKF->state, loopstart - timestart);
        comm->broadcast_message(settings.f("General", "broadcast_rate_div"));
        set_logtime(logtimes.commsend);

        EKF->predict();
        set_logtime(logtimes.predict);
        if (prediction_only) {
            state.vect = EKF->state_pred.vect;
            state.calculate();
        } else {
            EKF->correct();
        }
        set_logtime(logtimes.correct);
        if (settings.b("Logger", "log_times")) logger->log(logtimes);
        if (settings.b("Logger", "log_sensor")) logger->obj_log(*sensors);
        if (settings.b("Logger", "log_state")) logger->obj_log(EKF->state);
        if (settings.b("Logger", "log_R")) logger->log("R", state.R);
        if (settings.b("Logger", "log_state_pred")) logger->obj_log("Predicted ", EKF->state_pred);
        //savelog.log(EKF->state);
        if (logger->print(settings.f("Logger", "skip_every"))) {
            set_logtime(logtimes.log);
        }

        loopstart += looptime;
        std::this_thread::sleep_until(loopstart);
    }
}
