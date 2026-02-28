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
                                               * (CONTROL_STATE_SKIP_EVERY + 1));  // downsample for display
    auto timer = make_unique<CommTimer>(timestep_us, 2 * timestep_us);
    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    MessageInbox<ControlStateMsg, CONTROL_STATE_SKIP_EVERY> controlInbox(ControlStateMsgID, *controller_comm);
    unique_ptr<ControlStateMsg> controlState{new ControlStateMsg({})};
    MessageInbox<ControlInfoMsg> controlInfoInbox(ControlInfoMsgID, *controller_comm);
    unique_ptr<ControlInfoMsg> controlInfo{new ControlInfoMsg({})};
    MessageInbox<ControllerInfoMsg> controllerInfoInbox(ControllerInfoMsgID, *controller_comm);
    unique_ptr  <ControllerInfoMsg> controllerInfo{new ControllerInfoMsg({})};

    printf("Rx: ControlStateMsg %u bytes, ControlInfoMsg %u bytes\n",
           sizeof(ControlStateMsg) / sizeof(char), sizeof(ControlInfoMsg) / sizeof(char));
    std::cout<<"\tReading angles, T = "<< timestep_us << " us" << std::endl;
    while (true) {
        timer->wait_receive_message(controlInbox, *controlState);
        controlInfoInbox.get_newest(*controlInfo);
        controllerInfoInbox.get_newest(*controllerInfo);
        stdlogger->log("timestamp", controlState->sensor_data.timestamp_us);
        stdlogger->log("angles", controlState->sensor_data.angle, WALKER_COMM_NUM_MOTORS);
//        stdlogger->log("sleep_us", controlInfo->sleep_time_us);
        stdlogger->log("comm_rx_us", controlInfo->comm_rx_time_us);
        stdlogger->log("comm_tx_us", controlInfo->comm_tx_time_us);
        stdlogger->log("ADC_time_us", controlInfo->ADC_time_us);
        stdlogger->log("motor_set_time_us", controlInfo->motor_set_time_us);
        stdlogger->log("V_bat", controlInfo->battery_voltage);
        stdlogger->log("free bytes", controllerInfo->free_heap_bytes);
        stdlogger->log("cpu temp", controllerInfo->processor_temp);
        stdlogger->print();
    }
}
