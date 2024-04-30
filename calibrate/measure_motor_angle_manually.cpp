/* manually measure motor angles to get a calibration between

TODO:
    organize file logged data
    cmd line argument for settings filepath
    state send possible random disconnect error
*/
#include "comm_serial.hpp"
#include "messages.h"
#include "walkerUtils.hpp"
#include <memory>
#include <chrono>
#include <thread>

namespace chrono = std::chrono;


int main() {
    unique_ptr<SerialCommunicator> controller_comm(new SerialCommunicator(string("Controller_serial")));
    MessageInbox<ControlStateMsg> controlInbox(ControlStateMsgID, *controller_comm);
    unique_ptr<ControlStateMsg> controlState{new ControlStateMsg({})};
    chrono::milliseconds looptime(5);

    std::cout<<"\tReading angles, T = "<< looptime.count() << " ms" << std::endl;
    chrono::time_point<chrono::steady_clock> loopstart = chrono::steady_clock::now() + looptime;
    while (true) {
        std::this_thread::sleep_until(loopstart);
        controller_comm->receive_messages();
        if (controlInbox.get_newest(*controlState) >= 0) {
            std::cout << "\r" << controlState->sensor_data.timestamp_us << "   ";
            for (int i{}; i < WALKER_COMM_NUM_MOTORS; i++) {
                std::cout << controlState->sensor_data.angle[i] << "  ";
            }
            std::cout << std::flush;
        }
        loopstart += looptime;
    }
}
