#include <iostream>
#include <wiringSerial.h>
#include <unistd.h>
#include "maincomp_comm.hpp"

Communicator::Communicator(void) 
: num_inbox(num_messages), num_bad_bytes(bad_bytes), is_controller_connected(controller_connected),
  is_server_open(server_open), is_viewer_connected(viewer_connected) {
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

//thank you to https://github.com/Mad-Scientist-Monkey/sockets-ccpp-rpi

void Communicator::try_connect(void) {
    if (server_open) {
        std::cout<<"Trying to connect... "<<std::flush;
        listen(socket_desc, 3);
        int sock_length = sizeof(struct sockaddr_in);
        //Accept an incoming connection
        viewer_socket = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&sock_length);
        if (viewer_socket < 0)   return;
        viewer_connected = true;
        sendskipcntr = 0;
        std::cout<<"Connection accepted with "<<inet_ntoa(client.sin_addr)<<std::endl;
    }
}

void Communicator::start_server(int port_num, uint16_t msg_ID, size_t size, const char *msg) {
    //Create socket
    socket_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (socket_desc == -1) {
        std::cout<<"Could not create socket"<<std::endl;
        return;
    }
    //Prepare the sockaddr_in structure
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( port_num );
    //Bind
    if( bind(socket_desc,(struct sockaddr *)&server , sizeof(server)) < 0) std::cout<<"Server bind failed-";
    else server_open = true;
    std::cout<<"Server bind done"<<std::endl;
    viewer_ID = msg_ID;
    viewer_message = msg;
    viewer_size = size;
}

void Communicator::broadcast_message(unsigned skip_every) {
    if (viewer_connected) {
        if (sendskipcntr++ == skip_every) {
            write(viewer_socket, viewer_message, viewer_size);
            sendskipcntr = 0;
        }
    } else {
        try_connect();
    }
}
