/* Communication over TCP socket server
 * Dec 2022
 * TODO:
 *      receive messages
 *      sometimes, disconnecting causes it to print a lot of junk then segfault
 */

#ifndef SIMPLEWALKER_COMM_TCP_H
#define SIMPLEWALKER_COMM_TCP_H

#include <memory>
#include "communication.hpp"

struct TCPCommunicatorState;

class TCPCommunicator : public Communicator {
private:
    std::unique_ptr<TCPCommunicatorState> state;
public:
    explicit TCPCommunicator(const string &_name);
    ~TCPCommunicator() override;
    void start_server(int port_num);
    void stop_server();
    bool is_connected() const;
    inline bool receive_bytes() override;
    int send(const MessageBoxInterface &outbox, const char *data_start) override; //return 0 on success
};


#endif //SIMPLEWALKER_COMM_TCP_H
