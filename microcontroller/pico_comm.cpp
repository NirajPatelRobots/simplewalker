#include "pico_comm.hpp"
#include "pico/stdlib.h"


int PicoCommunication::send(const MessageBoxInterface &outbox, const char *data_start) {
    stdio_put_string(data_start, outbox.msg_len, false, false);
    stdio_flush();
    return 0;
}

PicoCommunication::PicoCommunication(const string &name)
    : Communicator(name) {
    stdio_init_all();
}

bool PicoCommunication::receive_bytes() {
    int inChar = getchar_timeout_us(timeout_us);
    int bytes_received = 0;
    while (inChar != PICO_ERROR_TIMEOUT
           && (max_bytes_per_receive == 0 || ++bytes_received <= max_bytes_per_receive)) {
        instream.push_back((char) inChar);
        inChar = getchar_timeout_us(timeout_us);
        bytes_received++;
    }
    return bytes_received > 0;
}
