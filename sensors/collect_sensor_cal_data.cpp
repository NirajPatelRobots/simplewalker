/* collect the data for sensor calibration 
TODO:
    delete old calibration files so we don't get confused (ansible?)
*/
#include "logger.hpp"
#include "comm_serial.hpp"
#include "messages.h"
#include <thread>


int main() {
    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    MessageInbox<ControlStateMsg_sns> controlInbox(ControlStateMsgID, *controller_comm);
    unique_ptr<ControlStateMsg_sns> controlState{new ControlStateMsg_sns({})};
    Logger savelog;
    const int NUM_DATA_NEEDED = 200;
    int log_num{0};
    string user_input{""};

    controller_comm->flush_message_queue(3, true);
    cout << "Start calibration. Keep still.    " << endl;
    while (user_input != "done") {
        savelog = Logger("data/stationary_calibration_" + std::to_string(log_num) + ".log");
        int num_data_got = 0;
        controller_comm->clear_buffer();
        controller_comm->flush_message_queue();
        while (num_data_got < NUM_DATA_NEEDED) {
            controller_comm->receive_messages();
            if (controlInbox.get_newest(*controlState) >= 0) {
                savelog.log(controlState->sensor_data);
                savelog.print();
                ++num_data_got;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        cout << "Move to a different position and press enter, or type done to stop ";
        std::getline(std::cin, user_input);
        ++log_num;
    }
}
