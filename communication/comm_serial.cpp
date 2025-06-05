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

bool SerialCommunicator::receive_bytes() {
    if (!serialDataAvail(serialfile)) return false;
    while (serialDataAvail(serialfile)) {
        instream.push_back( serialGetchar(serialfile) );
    }
    return true;
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
