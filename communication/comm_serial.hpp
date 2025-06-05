/* Serial communication for simplewalker
 * created Dec 2022
 * TODO:
 *      detects stream disruption
 *      switch to standard C/C++ serial library
 *      change device name
 */

#ifndef SIMPLEWALKER_COMM_SERIAL_H
#define SIMPLEWALKER_COMM_SERIAL_H

#include "communication.hpp"

class SerialCommunicator : public Communicator {
    int serialfile;
    bool is_connected_;
public:
    explicit SerialCommunicator(string _name);
    bool is_connected() const {return is_connected_;}
    bool receive_bytes() override;
    int send(const MessageBoxInterface &outbox, const char *data_start) override; //return 0 on success
    ~SerialCommunicator() override;
    void clear_buffer();
};


#endif //SIMPLEWALKER_COMM_SERIAL_H
