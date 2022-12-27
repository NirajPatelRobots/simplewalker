/* Communication over TCP socket
 * Dec 2022
 * TODO:
 *      receive messages
 *      maybe don't require include arpa/inet in header
 *      detects stream disruption
 *      wait for connection in another thread
 */

#ifndef SIMPLEWALKER_COMM_TCP_H
#define SIMPLEWALKER_COMM_TCP_H

#include <arpa/inet.h>
#include "communication.hpp"

class TCPCommunicator : public Communicator {
private:
    int file_desc, socket_desc;
    bool server_is_open_, is_connected_;
    struct sockaddr_in server, client;
public:
    TCPCommunicator(string _name);
    ~TCPCommunicator();
    void start_server(int port_num);
    void try_connect();
    void disconnect();
    bool server_is_open() {return server_is_open_;}
    bool is_connected() {return is_connected_;}
    void receive_messages() override; // TODO
    int send(const MessageBoxInterface &outbox, const char *data_start) override; //return 0 on success
};


#endif //SIMPLEWALKER_COMM_TCP_H
