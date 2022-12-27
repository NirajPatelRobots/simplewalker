#include "comm_serial.hpp"
#include <wiringSerial.h> // raspberry pi serial

SerialCommunicator::SerialCommunicator(string _name) : Communicator(_name) {
    serialfile = serialOpen("/dev/ttyACM0", 115200);
    is_connected_ = (serialfile < 0);
    clear_buffer();
}

SerialCommunicator::~SerialCommunicator() {
    serialClose(serialfile);
}

MessageBoxInterface* SerialCommunicator::parse_buffer_inbox() {
    if (instream.size() < 2) throw std::logic_error("Buffer doesn't have enough data for msgID");
    int16_t ID = ((int16_t)(instream[0] & 0xFF) + (int16_t)((instream[1] & 0xFF) << 8));
    MessageBoxInterface* inbox = get_inbox(ID);
    if (!inbox) {
        instream.pop_front();
        num_bad_bytes_in_++;
    }
    return inbox;
}

void SerialCommunicator::receive_messages() {
    MessageBoxInterface *inbox{};
    while (serialDataAvail(serialfile)) {
        instream.push_back( serialGetchar(serialfile) );
        try {
            inbox = parse_buffer_inbox();
        } catch (std::logic_error const&) {
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

int SerialCommunicator::send(const MessageBoxInterface &outbox, const char *data_start) {
    for (unsigned i = 0; i < outbox.msg_len; i++) {
        serialPutchar(serialfile, data_start[i]);
    }
    return 0;
}

void SerialCommunicator::clear_buffer() {
    serialFlush(serialfile);
}
