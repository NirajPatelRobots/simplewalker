/* measure and report motor angles
can use to calibrate motor settings (raw angle <-> angle in radians)

TODO:
 combine with monitor_only?
*/
#include "comm_serial.hpp"
#include "communication_timer.hpp"
#include "logger.hpp"
#include "messages.h"
#include <memory>


const static unsigned CONTROL_STATE_SKIP_EVERY {7};  // 30ms -> 240ms

int main() {
    WalkerSettings settings("settings/settings.xml");
    unsigned timestep_us = static_cast<unsigned>(1e6 * settings.f("General.main_timestep")
                                               * (CONTROL_STATE_SKIP_EVERY + 1)  // downsample for display and wi-fi
                                               * 1.0025);  // hack, was going too fast
    auto timer = make_unique<CommTimer>(timestep_us, 2 * timestep_us);
    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    MessageInbox<ControlStateMsg, CONTROL_STATE_SKIP_EVERY> controlInbox(ControlStateMsgID, *controller_comm);
    unique_ptr<ControlStateMsg> controlState{new ControlStateMsg({})};
    MessageInbox<ControlInfoMsg> controlInfoInbox(ControlInfoMsgID, *controller_comm);
    unique_ptr<ControlInfoMsg> controlInfo{new ControlInfoMsg({})};

    std::cout<<"\tReading angles, T = "<< timestep_us << " us" << std::endl;
    while (true) {
        timer->wait_receive_message(controlInbox, *controlState);
        controlInfoInbox.get_newest(*controlInfo);
        stdlogger->log("timestamp", controlState->sensor_data.timestamp_us);
        stdlogger->log("angles", controlState->sensor_data.angle, WALKER_COMM_NUM_MOTORS);
        stdlogger->log("runtime_us", controlInfo->control_runtime_us);
        stdlogger->log("no_comm_runtime", controlInfo->no_comm_runtime_us);
        stdlogger->log("V_bat", controlInfo->battery_voltage);
        stdlogger->print();
    }
}
