#include "comm_serial.hpp"
#include <wiringSerial.h> // raspberry pi serial

SerialCommunicator::SerialCommunicator(string _name) : Communicator(_name) {
    serialfile = serialOpen("/dev/ttyACM0", 115200);
    is_connected_ = (serialfile > 0);
    clear_buffer();
}

SerialCommunicator::~SerialCommunicator() {
    serialClose(serialfile);
}

MessageBoxInterface* SerialCommunicator::parse_buffer_inbox() {
    if (instream.size() < 2) throw std::logic_error("Buffer doesn't have enough data for msgID");
    int16_t ID = ((int16_t)instream.at(0) + ((int16_t)instream.at(1) << 8));
    MessageBoxInterface* inbox = get_inbox(ID);
    if (!inbox) {
        unexpected_bytes_in.push_back(instream.at(0));
        instream.pop_front();
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
