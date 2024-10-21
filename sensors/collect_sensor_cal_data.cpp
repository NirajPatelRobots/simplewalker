/* collect the data for sensor calibration 
TODO:
    detect when moved to different position
    check for IMUDataMsg->errcode
    temperature
*/
#include "logger.hpp"
#include "comm_serial.hpp"
#include "messages.h"
#include <thread>
#include <filesystem>

namespace fs = std::filesystem;

int main() {
    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    MessageInbox<IMUDataMsg> IMUInbox(IMUDataMsgID, *controller_comm);
    auto IMUData = make_unique<IMUDataMsg>();
    Logger savelog;
    const int NUM_DATA_NEEDED = 1024;
    const std::string STATIONARY_FILE_PREFIX = "stationary_calibration_";
    int log_num{0};
    string user_input{""};
    fs::path data_dir{"data"};
    if (!fs::exists(data_dir)) {
        data_dir = "../data";
        if (!fs::exists(data_dir)) {
            cout << "data/ directory not found. Run from simplewalker dir." << endl;
            return 1;
        }
    }
    for (const auto& entry : fs::directory_iterator(data_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".log"
            && entry.path().filename().string().find(STATIONARY_FILE_PREFIX) != std::string::npos) {
            fs::remove(entry.path());
        }
    }

    controller_comm->flush_message_queue(3, true);
    cout << "Start calibration. Keep still.    " << endl;
    while (user_input != "done") {
        savelog = Logger(data_dir / (STATIONARY_FILE_PREFIX + std::to_string(log_num) + ".log"));
        if (!savelog.is_saving_to_file()) {
            cout << "Couldn't create log file, exiting. Run from simplewalker dir." << endl;
            return 2;
        }
        int num_data_got = 0;
        controller_comm->clear_buffer();
        controller_comm->flush_message_queue(10);
        while (num_data_got < NUM_DATA_NEEDED) {
            controller_comm->receive_messages();
            while (IMUInbox.get_newest(*IMUData) >= 0) {
                savelog.log("utime", IMUData->timestamp_us);
                savelog.log("accel", IMUData->accel, 3);
                savelog.log("gyro", IMUData->gyro, 3);
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
