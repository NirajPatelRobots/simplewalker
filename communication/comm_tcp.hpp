/* Communication over TCP socket
 * Dec 2022
 * TODO:
 *      receive messages
 *      detects stream disruption
 *      wait for connection in another thread
 */

#ifndef SIMPLEWALKER_COMM_TCP_H
#define SIMPLEWALKER_COMM_TCP_H

#include <memory>
#include "communication.hpp"

struct TCPCommunicatorState;

class TCPCommunicator : public Communicator {
private:
    std::unique_ptr<TCPCommunicatorState> state;
    bool server_is_open_, is_connected_;
public:
    explicit TCPCommunicator(string _name);
    ~TCPCommunicator() override;
    void start_server(int port_num);
    void try_connect();
    void disconnect();
    bool server_is_open() const {return server_is_open_;}
    bool is_connected() const {return is_connected_;}
    void receive_messages() override; // TODO
    int send(const MessageBoxInterface &outbox, const char *data_start) override; //return 0 on success
};


#endif //SIMPLEWALKER_COMM_TCP_H
