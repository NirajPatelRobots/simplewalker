/* Simplewalker main program

TODO:
    organize file logged data
    store bools for whether to do each logger->log instead of rereading settings
    create common shared code:
        - RobotMicroInterface in monitor_only
        - Sensors and State Estimation with settings
    cmd line argument for settings filepath
*/
#include "comm_serial.hpp"
#include "comm_tcp.hpp"
#include "messages.h"
#include "state_estimation.hpp"
#include "convenientLogger.hpp"
#include "communication_timer.hpp"

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
    MessageInbox<ControlInfoMsg> controlInfoInbox(ControlInfoMsgID, *controller_comm);
    unique_ptr  <ControlInfoMsg> controlInfo{new ControlInfoMsg({})};
    MessageInbox<ControllerInfoMsg> controllerInfoInbox(ControllerInfoMsgID, *controller_comm);
    unique_ptr  <ControllerInfoMsg> controllerInfo{new ControllerInfoMsg({})};
    MessageOutbox<RobotStateMsg> stateOutbox(RobotStateMsgID, *base_comm);

    unique_ptr<SensorBoss> sensors(new SensorBoss(settings.f("SensorBoss", "accel_stddev"),
                                                  settings.f("SensorBoss", "gyro_stddev")));
    sensors->set_bias(settings.vf("SensorBoss", "accel_bias"), settings.vf("SensorBoss", "gyro_bias"));
    sensors->set_IMU_orientation(settings.vf("SensorBoss", "IMU_orientation"));
    float timestep = settings.f("General", "main_timestep");
    auto timestep_us = static_cast<unsigned>(1e6 * timestep);
    const bool prediction_only = settings.b("State_Estimation", "prediction_only");
    if (prediction_only) std::cout<<"\tPrediction Only, Ignoring Sensors"<<std::endl;
    unique_ptr<StateEstimator> EKF(new StateEstimator(timestep,
                                   settings.f("State_Estimation", "pos_stddev"), 
                                   settings.f("State_Estimation", "axis_stddev"),
                                   settings.f("State_Estimation", "vel_stddev"),
                                   settings.f("State_Estimation", "angvel_stddev")));
    EKF->set_damping_deceleration(settings.f("State_Estimation", "damping_deceleration"));
    RobotState &state = EKF->state;
    shared_ptr<ConvenientLogger> logger{std::static_pointer_cast<ConvenientLogger>(stdlogger)};
    ConvenientLogger savelog("data/statelog.log");
    Logtimes logtimes{};
    auto timer = make_unique<CommTimer>(timestep_us, MSG_WAIT_TIME_US);

    if (settings.b("General", "state_send")) base_comm->start_server(settings.f("General", "state_send_port"));
    controller_comm->flush_message_queue(settings.i("General", "initial_flush_controller_queue"), true);

    int logger_skip_every = settings.f("Logger.skip_every");
    cout << "Timestep: " << timestep_us << " us\nPrint interval: " << timestep_us * (logger_skip_every + 1) << " us\n";
    int broadcast_counter = 0;
    while (true) {
        timer->wait_receive_message(controlInbox, *controlState);
        controlInfoInbox.get_newest(*controlInfo);
        controllerInfoInbox.get_newest(*controllerInfo);
        set_logtime(logtimes.sleep);
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
        if (++broadcast_counter >= settings.i("General.broadcast_rate_div")) {
            set_state_msg(&stateOutbox.message, EKF->state, timer->duration());
            stateOutbox.send();
            broadcast_counter = 0;
        }
        set_logtime(logtimes.commsend);

        if (settings.b("Logger", "log_times")) logger->log(logtimes);
        if (settings.b("Logger", "log_controlstate")) logger->log("controlstate", (float*)controlState.get(), sizeof(ControlStateMsg) / sizeof(float));
        if (settings.b("Logger", "log_sensor")) logger->obj_log(sensors->data);
        if (settings.b("Logger", "log_sensor_pred")) logger->obj_log(sensors->data_pred);
        if (settings.b("Logger", "log_state")) logger->obj_log(EKF->state);
        if (settings.b("Logger", "log_R")) logger->log("R", state.R);
        if (settings.b("Logger", "log_state_pred")) logger->obj_log("Predicted ", EKF->state_pred);
        if (settings.b("Logger", "log_free_bytes")) stdlogger->log("free bytes", controllerInfo->free_heap_bytes);
        if (settings.b("Logger", "log_cpu_temp")) stdlogger->log("cpu temp", controllerInfo->processor_temp);
        if (settings.b("Logger", "log_unexpected_comm_data")) logger->log("unexpected comm data", controller_comm->unexpected_bytes_in);
        savelog.log(logtimes);
        savelog.obj_log(EKF->state);
        savelog.obj_log(*sensors);
        savelog.print(settings.f("Logger", "skip_every"));
        if (logger->print(settings.f("Logger", "skip_every"))) {
            set_logtime(logtimes.log);
        }
    }
}
