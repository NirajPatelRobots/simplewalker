#include "pico_comm.hpp"
#include <stdio.h>
#include "algorithm"


void PicoCommunication::receive_messages() {
    MessageBoxInterface *inbox{};
    while (true) {
        int inChar = getchar_timeout_us(timeout_us);
        if (inChar == PICO_ERROR_TIMEOUT) {
            return;
        }
        instream.push_back( (char)inChar );
        try {
            inbox = parse_buffer_inbox();
        } catch (std::logic_error const&) {
            if (instream.size() >= 2) instream.pop_front();
            continue;
        }
        if (inbox && instream.size() >= inbox->msg_len) {
            inbox->set_message(instream.begin(), instream.begin() + inbox->msg_len);
            for (unsigned i = 0; i < inbox->msg_len; i++) {
                instream.pop_front();
            }
        }
    }
}

int PicoCommunication::send(const MessageBoxInterface &outbox, const char *data_start) {
    for (int i = 0; i < outbox.msg_len; i++) {
        putchar_raw(data_start[i]);
    }
}

PicoCommunication::PicoCommunication(string name)
    : instream{}, Communicator(name) {
    stdio_init_all();
}

MessageBoxInterface* PicoCommunication::parse_buffer_inbox() {
    if (instream.size() < 2) throw std::logic_error("Buffer doesn't have enough data for msgID");
    int16_t ID = ((int16_t)(instream[0] & 0xFF) + (int16_t)((instream[1] & 0xFF) << 8));
    MessageBoxInterface* inbox = get_inbox(ID);
    if (!inbox) {
        if (std::find(unexpected_IDs.begin(), unexpected_IDs.end(), ID) == unexpected_IDs.end())
            unexpected_IDs.push_back(ID);
        instream.pop_front();
        num_bad_bytes_in_++;
    }
    return inbox;
}
