/* Simplewalker main program

*/
#include "maincomp_comm.hpp"
#include "state_estimation.hpp"
#include <iostream>
#include <chrono>
#include <thread>

namespace chrono = std::chrono;
#define LOOP_TIME_MS 30
#define MSG_WAIT_TIME_US 1000

int main() {
    ControlStateMsg *controlstate = new ControlStateMsg;
    controlstate->ID = 0;
    Communicator comm(ControlStateMsgID, sizeof(ControlStateMsg),
                      ControlTargetMsgID, sizeof(ControlTargetMsg));
    // EKF parameters
    float accel_stddev = 1.0, gyro_stddev = 0.01, timestep = 0.03,
        pos_stddev = 0.01, euler_stddev = 0.05, vel_stddev = 1.0, angvel_stddev = 1.0;
    StateEstimator *EKF = new StateEstimator(accel_stddev, gyro_stddev, timestep, 
                                    pos_stddev, euler_stddev, vel_stddev, angvel_stddev);

    Eigen::Map<Vector3f> accel_data(controlstate->accel), gyro_data(controlstate->gyro);

    chrono::milliseconds looptime(LOOP_TIME_MS);
    chrono::microseconds msgwaittime(MSG_WAIT_TIME_US);
    chrono::time_point<chrono::steady_clock> readtime;
    bool ERR_msg_late = false;
    int num_msgs = 0;

    std::cout<<"Time    | BAD | "<<state_CSV_header()<<"\n";
    auto loopstart = chrono::steady_clock::now();
    while (true) {
        comm.handle_messages();
        while ((num_msgs = comm.read_message((char *)controlstate)) < 0 && readtime - loopstart < msgwaittime) {
            comm.handle_messages();
            readtime = chrono::steady_clock::now();
        }
        ERR_msg_late = (readtime > loopstart);
        if (ERR_msg_late) {
            std::cout<<"late\n";
            loopstart = readtime; // slow down time so we don't get ahead
        }

        EKF->predict();
        EKF->correct(accel_data, gyro_data);

        std::cout<<"\r"<<controlstate->timestamp_us<<" | "<<comm.num_bad_bytes<<" | "<<EKF->state<<"  |";
        /*       <<"  |  "<<controlstate->accel[0]<<' '<<controlstate->accel[1]<<' '<<controlstate->accel[2]
                 <<"  |  "<<controlstate->gyro[0]<<' '<<controlstate->gyro[1]<<' '<<controlstate->gyro[2]<<std::flush;*/
        loopstart += looptime;
        std::this_thread::sleep_until(loopstart);
    }
}
