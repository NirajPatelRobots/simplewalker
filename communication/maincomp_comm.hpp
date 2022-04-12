/*main computer (Rpi zero) communication with the microcontroller (pi pico)
TODO: 
    detects stream disruption
    receive multiple IDs
    send wifi to desktop
*/
#include <queue>
#include <stdint.h>
#include "messages.h"


class Communicator {
    int serialfile;
    uint16_t read_ID, send_ID;
    unsigned int read_size, send_size, num_messages, bad_bytes, _readindex;
    bool controller_connected;
    std::queue<char> instream;
public:
    const unsigned int &num_inbox, &num_bad_bytes;
    const bool &is_controller_connected;
    Communicator(void);
    Communicator(uint16_t listen_ID, size_t listen_size);
    Communicator(uint16_t listen_ID, size_t listen_size, uint16_t send_ID, size_t send_size);
    ~Communicator(void);
    void set_listen_ID(uint16_t ID, size_t size);
    int read_message(char *buff); //returns how many messages there are after this one. -1 is a failure.
    void set_send_ID(uint16_t ID, size_t size);
    void send_message(const char *buff);
    void handle_messages(void);
};
