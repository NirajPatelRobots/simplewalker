/* Simplewalker main program

TODO:
    BUG: Sometimes, correlated with receiving message late first, state estimation takes 30 ms. 
    organize file logged data
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

void start_control_communication(Communicator &comm, ControlStateMsg_sns *controlstate, RobotStateMsg *sendstate) {
    controlstate->ID = 0;
    sendstate->ID = RobotStateMsgID;
    sendstate->errcode = 0;
    for (int i = 0; i < 6; ++i) {sendstate->leg_pos[i] = 0; sendstate->leg_vel[i] = 0;}
    std::cout<<"Waiting for controller messages...\r";
    int num_msgs = 0;
    while (num_msgs < 3) {
        comm.handle_messages();
        if (comm.read_message((char *)controlstate) >= 0)  ++num_msgs;
    }
}


int main() {
    WalkerSettings settings("settings/settings.xml"); //settings
    
    unique_ptr<ControlStateMsg_sns> controlstate(new ControlStateMsg_sns);
    unique_ptr<RobotStateMsg> sendstate(new RobotStateMsg);

    unique_ptr<Communicator> comm(new Communicator(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg)));

    unique_ptr<SensorBoss> sensors(new SensorBoss(settings.f("SensorBoss", "accel_stddev"),
                                                  settings.f("SensorBoss", "gyro_stddev")));
    sensors->set_bias(settings.vf("SensorBoss", "accel_bias"), settings.vf("SensorBoss", "gyro_bias"));
    float timestep = settings.f("General", "main_timestep");
    const bool prediction_only = settings.b("State_Estimation", "prediction_only");
    if (prediction_only) std::cout<<"\tPrediction Only, Ignoring Sensors"<<std::endl;
    unique_ptr<StateEstimator> EKF(new StateEstimator(timestep,
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
    ConvenientLogger savelog("data/statelog.log");
    Logtimes logtimes;
    
    comm->start_server(settings.f("General", "state_send_port"), RobotStateMsgID,
                        sizeof(RobotStateMsg), (const char *)sendstate.get());
    if (settings.b("General", "state_send")) comm->try_connect();
    start_control_communication(*comm, controlstate.get(), sendstate.get());

    std::cout<<"    Start main loop, T = "<< looptime.count() << " ms     " << std::endl;
    chrono::time_point<chrono::steady_clock> timestart = chrono::steady_clock::now();
    chrono::time_point<chrono::steady_clock> loopstart = timestart + looptime;
    std::this_thread::sleep_until(loopstart);
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

        EKF->predict();
        set_logtime(logtimes.predict);
        sensors->update_sensors(&controlstate->sensor_data);
        sensors->predict(EKF->state_pred, state, EKF->dt);
        set_logtime(logtimes.sensorboss);
        if (prediction_only) {
            state.vect = EKF->state_pred.vect;
            state.calculate();
        } else {
            EKF->correct(*sensors);
        }
        set_logtime(logtimes.correct);
        set_state_msg(sendstate.get(), EKF->state, loopstart - timestart);
        comm->broadcast_message(settings.f("General", "broadcast_rate_div"));
        set_logtime(logtimes.commsend);

        if (settings.b("Logger", "log_times")) logger->log(logtimes);
        if (settings.b("Logger", "log_controlstate")) logger->log("controlstate", (float*)controlstate.get(), sizeof(ControlStateMsg) / sizeof(float));
        if (settings.b("Logger", "log_sensor")) logger->log(sensors->data);
        if (settings.b("Logger", "log_sensor_pred")) logger->log(sensors->data_pred);
        if (settings.b("Logger", "log_state")) logger->obj_log(EKF->state);
        if (settings.b("Logger", "log_R")) logger->log("R", state.R);
        if (settings.b("Logger", "log_state_pred")) logger->obj_log("Predicted ", EKF->state_pred);
        savelog.log(logtimes);
        savelog.obj_log(EKF->state);
        savelog.obj_log(*sensors);
        savelog.print(settings.f("Logger", "skip_every"));
        if (logger->print(settings.f("Logger", "skip_every"))) {
            set_logtime(logtimes.log);
        }

        loopstart += looptime;
        std::this_thread::sleep_until(loopstart);
    }
}
