/*pico 2 way communication with the raspberry pi zero (main computer)
 TODO:
    uart
    close stream
    assumes LSB first
*/
#ifndef SIMPLEWALKER_PICO_COMM_HPP
#define SIMPLEWALKER_PICO_COMM_HPP
#include "../communication/communication.hpp"

class PicoCommunication : public Communicator {
public:
    int timeout_us{10};
    int max_bytes_per_receive{256}; // 0 means until timeout
    explicit PicoCommunication(const string &name); // wraps stdio_init_all()
    PicoCommunication() : PicoCommunication("Pico Communication") {}
    ~PicoCommunication() override = default; // cannot be cleaned up without disrupting communication
    bool receive_bytes() override;
    int send(const MessageBoxInterface &outbox, const char *data_start) override; //return 0 on success
};
#endif  //SIMPLEWALKER_PICO_COMM_HPP
