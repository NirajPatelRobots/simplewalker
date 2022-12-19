/* Serial communication for simplewalker
 * created Dec 2022
 * TODO:
 *      detects stream disruption
 */

#ifndef SIMPLEWALKER_COMM_SERIAL_H
#define SIMPLEWALKER_COMM_SERIAL_H

#include <queue>
#include "communication.hpp"

class SerialCommunicator : public Communicator {
    int serialfile;
    bool is_connected_;
    deque<char> instream;
    MessageBoxInterface *parse_buffer_inbox();
public:
    explicit SerialCommunicator(string _name);
    bool is_connected() {return is_connected_;}
    void receive_messages() override;
    int send(const MessageBoxInterface &outbox, char *data_start) override; //return 0 on success
    ~SerialCommunicator();
    void clear_buffer();
};


#endif //SIMPLEWALKER_COMM_SERIAL_H
