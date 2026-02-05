/* Monitor diagnostic messages on base station machine
TODO:
    send controllerStateMsg?
    create common shared code:
        - RobotMicroInterface
*/
#include "communication_timer.hpp"
#include "logger.hpp"
#include "messages.h"
#include "settings.hpp"
#include "comm_serial.hpp"
#include "comm_tcp.hpp"


const static unsigned MSG_WAIT_TIME_US {1000000};  //wait this long before skipipng
const static unsigned IMU_DATA_SKIP_EVERY {49};  // 200Hz -> 4Hz


// Group micro messages, forward to base_comm. Does its job just by existing.
class RobotMicroInterface {
public:
    unique_ptr<SerialCommunicator> controller_comm;
    MessageInbox<IMUDataMsg, IMU_DATA_SKIP_EVERY> IMUDataInbox;
    MessageInbox<IMUInfoMsg>      IMUInfoInbox;
    MessageInbox<ControlStateMsg> controlInbox;
    unique_ptr  <IMUDataMsg>      IMUData;
    unique_ptr  <IMUInfoMsg>      IMUInfo;
    unique_ptr  <ControlStateMsg> controlState;
    RobotMicroInterface(TCPCommunicator *base_comm) :
            controller_comm(new SerialCommunicator(string("Controller_serial"))),
            IMUDataInbox(IMUDataMsgID, *controller_comm, base_comm),
            IMUInfoInbox(IMUInfoMsgID, *controller_comm, base_comm),
            controlInbox(ControlStateMsgID, *controller_comm),
            IMUData(new IMUDataMsg({})),
            IMUInfo(new IMUInfoMsg({})),
            controlState{new ControlStateMsg({})}
    {}
};


int main() {
    WalkerSettings settings("settings/settings.xml");
    unsigned timestep_us = static_cast<unsigned>(1e6 * settings.f("General.main_timestep")
                                                 * (IMU_DATA_SKIP_EVERY + 1)  // downsample for display and wi-fi
                                                 * 1.0025);  // hack, was going too fast
    unsigned single_message_timestep_us = static_cast<unsigned>(1e6 * settings.f("General.main_timestep"));

    auto timer = make_unique<CommTimer>(timestep_us, MSG_WAIT_TIME_US, single_message_timestep_us);

    unique_ptr<TCPCommunicator> base_comm(new TCPCommunicator("Base_TCP"));
    RobotMicroInterface microInterface{base_comm.get()};

    base_comm->start_server(settings.f("General.state_send_port"));
    microInterface.controller_comm->flush_message_queue(settings.i("General.initial_flush_controller_queue"), true);

    int logger_skip_every = settings.f("Logger.skip_every");
    cout << "Timestep: " << timestep_us << " us\nPrint interval: " << timestep_us * (logger_skip_every + 1) << " us\n";

    while (true) {
        auto last_t = microInterface.IMUData->timestamp_us;
        timer->wait_receive_message(microInterface.IMUDataInbox, *microInterface.IMUData);

        microInterface.IMUInfoInbox.get_newest(*microInterface.IMUInfo);

        stdlogger->log("t", microInterface.IMUData->timestamp_us - last_t);
        stdlogger->log("dt", microInterface.IMUData->timestamp_us - last_t);
        stdlogger->log("accel[3]", microInterface.IMUData->accel, 3);
        stdlogger->log("gyro[3]", microInterface.IMUData->gyro, 3);
        stdlogger->log("unexpected comm data", microInterface.controller_comm->unexpected_bytes_in);
        stdlogger->print(logger_skip_every);
    }
}
