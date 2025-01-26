/*pico 2 way communication with the rasberry pi zero (main computer)
(LSB first)
*/
#ifndef SIMPLEWALKER_PICO_COMM_HPP
#define SIMPLEWALKER_PICO_COMM_HPP
#include "pico/stdlib.h"
#include "../communication/communication.hpp"

class PicoCommunication : public Communicator {
    deque<char> instream{};
    MessageBoxInterface *parse_buffer_inbox();
public:
    int timeout_us{1000};
    int max_bytes_per_receive{256};
    explicit PicoCommunication(string name); // wraps stdio_init_all()
    PicoCommunication() : PicoCommunication("Pico Communication") {}
    ~PicoCommunication() override = default; // cannot be cleaned up without disrupting communication
    void receive_messages() override;
    int send(const MessageBoxInterface &outbox, const char *data_start) override; //return 0 on success
};
#endif  //SIMPLEWALKER_PICO_COMM_HPP
