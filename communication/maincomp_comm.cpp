#include <stdio.h>
#include <wiringSerial.h>
#include "maincomp_comm.hpp"

Communicator::Communicator(void) 
: num_inbox(num_messages), num_bad_bytes(bad_bytes), is_controller_connected(controller_connected) {
    serialfile = serialOpen("/dev/ttyACM0", 115200);
    serialFlush(serialfile);
    controller_connected = (serialfile < 0);
    num_messages = 0;
    bad_bytes = 0;
    send_ID = 0;
    _readindex = 0;
}

Communicator::Communicator(uint16_t listen_ID, size_t listen_size)
: Communicator() {
    this->set_listen_ID(listen_ID, listen_size);
}

Communicator::Communicator(uint16_t listen_ID, size_t listen_size, uint16_t send_ID, size_t send_size)
: Communicator() {
    this->set_listen_ID(listen_ID, listen_size);
    this->set_send_ID(send_ID, send_size);
}

Communicator::~Communicator() {
    serialClose(serialfile);
}

void Communicator::set_listen_ID(uint16_t ID, size_t size) {
    read_ID = ID;
    read_size = size;
    num_messages = 0;
    _readindex = 0;
}

void Communicator::set_send_ID(uint16_t ID, size_t size) {
    send_ID = ID;
    send_size = size;
}

int Communicator::read_message(char *buff) {
    if (num_messages > 0) {
        for (unsigned i = 2; i < read_size; i++) { //start at 2 doesn't copy ID
            buff[i] = instream.front();
            instream.pop();
        }
        return --num_messages;
    }
    return -1;
}

void Communicator::handle_messages(void) {
    while (serialDataAvail(serialfile)) {
        if (_readindex < 2) { //looking for ID
            if (((char *)&read_ID)[_readindex] == serialGetchar(serialfile)) {
                _readindex++; //got part of the ID we want
            } else {
                bad_bytes++;
            }
        } else { // reading message
            instream.push( serialGetchar(serialfile) );
            if (++_readindex >= read_size) { //done
                _readindex = 0;
                num_messages++;
            }
        }
    }
}

void Communicator::send_message(const char *buff) {
    for (unsigned i = 0; i < send_size; i++) {
        serialPutchar(serialfile, buff[i]);
    }
}
