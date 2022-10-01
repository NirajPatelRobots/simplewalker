/* collect the data for sensor calibration 
TODO:
    combine start_control_communication with code in simplewalker.cpp, in communication message box
    sleep so don't overwhelm processor?
*/
#include "walkerUtils.hpp"
#include "logger.hpp"
#include "maincomp_comm.hpp"

void start_control_communication(Communicator &comm, ControlStateMsg_sns *controlstate) {
    controlstate->ID = 0;
    std::cout<<"Waiting for controller messages...\r";
    int num_msgs = 0;
    while (num_msgs < 3) {
        comm.handle_messages();
        if (comm.read_message((char *)controlstate) >= 0)  ++num_msgs;
    }
}

int main() {    
    unique_ptr<ControlStateMsg_sns> controlstate(new ControlStateMsg_sns);
    unique_ptr<Communicator> comm(new Communicator(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg)));
    Logger savelog;
    const int NUM_DATA_NEEDED = 200;
    int log_num{0};
    string user_input{""};

    start_control_communication(*comm, controlstate.get());
    cout << "Start calibration. Keep still.    " << endl;
    while (user_input != "done") {
        savelog = Logger("data/stationary_calibration_" + std::to_string(log_num) + ".log");
        int num_data_got = 0;
        while (num_data_got < NUM_DATA_NEEDED) {
            comm->handle_messages();
            if (comm->read_message((char *)controlstate.get()) >= 0) {
                savelog.log(controlstate->sensor_data);
                savelog.print();
                ++num_data_got;
            }
        }
        cout << "Move to a different position and press enter, or type done to stop ";
        std::getline(std::cin, user_input);
        ++log_num;
    }
}
