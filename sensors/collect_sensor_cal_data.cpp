/* collect the data for sensor calibration 
TODO:
    combine start_control_communication with code in simplewalker.cpp, in communication message box
    sleep so don't overwhelm processor?
    delete old calibration files so we don't get confused (ansible?)
*/
#include "walkerUtils.hpp"
#include "logger.hpp"
#include "comm_serial.hpp"
#include "messages.h"

void start_control_communication(MessageInbox<ControlStateMsg_sns> &controlInbox) {
    std::cout<<"Waiting for controller messages...\r" << std::flush;
    while (controlInbox.num_available() < 3) {
        controlInbox.comm.receive_messages();
    }
    controlInbox.clear();
}

int main() {
    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    MessageInbox<ControlStateMsg_sns> controlInbox(ControlStateMsgID, *controller_comm);
    unique_ptr<ControlStateMsg_sns> controlState{new ControlStateMsg_sns({})};
    Logger savelog;
    const int NUM_DATA_NEEDED = 200;
    int log_num{0};
    string user_input{""};

    start_control_communication(controlInbox);
    cout << "Start calibration. Keep still.    " << endl;
    while (user_input != "done") {
        savelog = Logger("data/stationary_calibration_" + std::to_string(log_num) + ".log");
        int num_data_got = 0;
        do {controller_comm->receive_messages();}
            while (controlInbox.get_newest(*controlState) > 0); // flush incoming messages
        while (num_data_got < NUM_DATA_NEEDED) {
            controller_comm->receive_messages();
            if (controlInbox.get_newest(*controlState) >= 0) {
                savelog.log(controlState->sensor_data);
                savelog.print();
                ++num_data_got;
            }
        }
        cout << "Move to a different position and press enter, or type done to stop ";
        std::getline(std::cin, user_input);
        ++log_num;
    }
}
