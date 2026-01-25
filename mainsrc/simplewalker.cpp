/* Simplewalker main program

TODO:
    organize file logged data
    cmd line argument for settings filepath
    state send possible random disconnect error
*/
#include "comm_serial.hpp"
#include "comm_tcp.hpp"
#include "messages.h"
#include "state_estimation.hpp"
#include "convenientLogger.hpp"
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

    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    unique_ptr<TCPCommunicator> base_comm(new TCPCommunicator(string("Base_TCP")));
    MessageInbox<ControlStateMsg> controlInbox(ControlStateMsgID, *controller_comm);
    unique_ptr<ControlStateMsg> controlState{new ControlStateMsg({})};
    MessageOutbox<RobotStateMsg> stateOutbox(RobotStateMsgID, *base_comm);

    unique_ptr<SensorBoss> sensors(new SensorBoss(settings.f("SensorBoss", "accel_stddev"),
                                                  settings.f("SensorBoss", "gyro_stddev")));
    sensors->set_bias(settings.vf("SensorBoss", "accel_bias"), settings.vf("SensorBoss", "gyro_bias"));
    sensors->set_IMU_orientation(settings.vf("SensorBoss", "IMU_orientation"));
    float timestep = settings.f("General", "main_timestep");
    const bool prediction_only = settings.b("State_Estimation", "prediction_only");
    if (prediction_only) std::cout<<"\tPrediction Only, Ignoring Sensors"<<std::endl;
    unique_ptr<StateEstimator> EKF(new StateEstimator(timestep,
                                   settings.f("State_Estimation", "pos_stddev"), 
                                   settings.f("State_Estimation", "axis_stddev"),
                                   settings.f("State_Estimation", "vel_stddev"),
                                   settings.f("State_Estimation", "angvel_stddev")));
    EKF->set_damping_deceleration(settings.f("State_Estimation", "damping_deceleration"));
    RobotState &state = EKF->state;
    chrono::milliseconds looptime(static_cast<int>(1000 * timestep));
    chrono::microseconds msgwaittime(MSG_WAIT_TIME_US);
    chrono::time_point<chrono::steady_clock> latereadtime;
    bool ERR_msg_late = false;
    shared_ptr<ConvenientLogger> logger{std::static_pointer_cast<ConvenientLogger>(stdlogger)};
    ConvenientLogger savelog("data/statelog.log");
    Logtimes logtimes{};

    if (settings.b("General", "state_send")) base_comm->start_server(settings.f("General", "state_send_port"));
    controller_comm->flush_message_queue(3, true);

    std::cout<<"    Start main loop, T = "<< looptime.count() << " ms     " << std::endl;
    chrono::time_point<chrono::steady_clock> timestart = chrono::steady_clock::now();
    chrono::time_point<chrono::steady_clock> loopstart = timestart + looptime;
    std::this_thread::sleep_until(loopstart);
    while (true) {
        set_logtime(logtimes.sleep);
        controller_comm->receive_messages();
        while (controlInbox.get_newest(*controlState) < 0) {
            controller_comm->receive_messages();
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
        sensors->update_sensors(&controlState->sensor_data);
        sensors->predict(EKF->state_pred, EKF->dt);
        set_logtime(logtimes.sensorboss);
        if (prediction_only) {
            state.vect = EKF->state_pred.vect;
            state.calculate();
        } else {
            EKF->correct(*sensors);
        }
        set_logtime(logtimes.correct);
        if ((loopstart - timestart).count() % ((int)settings.f("General", "broadcast_rate_div") * 100) < 100) {
            set_state_msg(&stateOutbox.message, EKF->state, loopstart - timestart);
            stateOutbox.send();
        }
        set_logtime(logtimes.commsend);

        if (settings.b("Logger", "log_times")) logger->log(logtimes);
        if (settings.b("Logger", "log_controlstate")) logger->log("controlstate", (float*)controlState.get(), sizeof(ControlStateMsg) / sizeof(float));
        if (settings.b("Logger", "log_sensor")) logger->obj_log(sensors->data);
        if (settings.b("Logger", "log_sensor_pred")) logger->obj_log(sensors->data_pred);
        if (settings.b("Logger", "log_state")) logger->obj_log(EKF->state);
        if (settings.b("Logger", "log_R")) logger->log("R", state.R);
        if (settings.b("Logger", "log_state_pred")) logger->obj_log("Predicted ", EKF->state_pred);
        if (settings.b("Logger", "log_unexpected_comm_data")) logger->log("unexpected comm data", controller_comm->unexpected_bytes_in);
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
