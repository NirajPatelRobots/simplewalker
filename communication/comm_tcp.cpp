#include "comm_tcp.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>

//thank you to https://github.com/Mad-Scientist-Monkey/sockets-ccpp-rpi

struct TCPCommunicatorState {
    struct sockaddr_in server{}, client{};
    int file_desc{}, socket_desc{};
};

TCPCommunicator::TCPCommunicator(std::string _name)
    : Communicator(_name), state(std::make_unique<TCPCommunicatorState>()),
    server_is_open_(false), is_connected_(false) {}

TCPCommunicator::~TCPCommunicator() {
    disconnect();
}

void TCPCommunicator::start_server(int port_num) {
    state->file_desc = socket(AF_INET , SOCK_STREAM , 0);
    if (state->file_desc == -1) {
        std::cout<<"Could not create socket"<<std::endl;
        return;
    }
    state->server.sin_family = AF_INET;
    state->server.sin_addr.s_addr = INADDR_ANY;
    state->server.sin_port = htons( port_num );
    if( bind(state->file_desc,(struct sockaddr *)&(state->server) , sizeof(state->server)) < 0)
        std::cout<<"Server bind failed-"<<std::endl;
    else server_is_open_ = true;
}

void TCPCommunicator::try_connect() {
    if (server_is_open()) {
        std::cout<<"Trying to connect... "<<std::flush;
        listen(state->file_desc, 3);
        int sock_length = sizeof(struct sockaddr_in);
        state->socket_desc = accept(state->file_desc, (struct sockaddr *)&state->client, (socklen_t*)&sock_length);
        if (state->socket_desc < 0)   return;
        is_connected_ = true;
        std::cout<<"Connection accepted with "<<inet_ntoa(state->client.sin_addr)<<std::endl;
    } else {
        std::cout<<"Server not opened"<<std::endl;
    }
}

void TCPCommunicator::disconnect() {
    close(state->file_desc);
    is_connected_ = false;
    server_is_open_ = false;
}

int TCPCommunicator::send(const MessageBoxInterface &outbox, const char *data_start) {
    if (is_connected()) {
        if (write(state->socket_desc, data_start, outbox.msg_len) == (ssize_t)outbox.msg_len)
            return 0;
        return 1;
    }
    return 2;
}

