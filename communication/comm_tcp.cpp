#include "comm_tcp.hpp"
#include <sys/socket.h>
#include <unistd.h>

//thank you to https://github.com/Mad-Scientist-Monkey/sockets-ccpp-rpi

TCPCommunicator::TCPCommunicator(std::string _name)
    : Communicator(_name), file_desc(0), socket_desc(0),
    server_is_open_(false), is_connected_(false) {}

TCPCommunicator::~TCPCommunicator() {
    disconnect();
}

void TCPCommunicator::start_server(int port_num) {
    file_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (file_desc == -1) {
        std::cout<<"Could not create socket"<<std::endl;
        return;
    }
    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons( port_num );
    if( bind(file_desc,(struct sockaddr *)&server , sizeof(server)) < 0)
        std::cout<<"Server bind failed-"<<std::endl;
    else server_is_open_ = true;
}

void TCPCommunicator::try_connect() {
    if (server_is_open()) {
        std::cout<<"Trying to connect... "<<std::flush;
        listen(file_desc, 3);
        int sock_length = sizeof(struct sockaddr_in);
        socket_desc = accept(file_desc, (struct sockaddr *)&client, (socklen_t*)&sock_length);
        if (socket_desc < 0)   return;
        is_connected_ = true;
        std::cout<<"Connection accepted with "<<inet_ntoa(client.sin_addr)<<std::endl;
    } else {
        std::cout<<"Server not opened"<<std::endl;
    }
}

void TCPCommunicator::disconnect() {
    close(file_desc);
    is_connected_ = false;
    server_is_open_ = false;
}

void TCPCommunicator::receive_messages() {
    // TODO
}

int TCPCommunicator::send(const MessageBoxInterface &outbox, const char *data_start) {
    if (is_connected()) {
        if (write(socket_desc, data_start, outbox.msg_len) == (const ssize_t)outbox.msg_len)
            return 0;
        return 1;
    }
    return 2;
}

