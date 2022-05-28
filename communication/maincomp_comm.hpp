/*main computer (Rpi zero) communication with the microcontroller (pi pico)
TODO: 
    detects stream disruption
    receive multiple IDs
    make pointers safe
    BUG: messages to desktop have random settings text?
    wait for connection in another thread
*/
#include <queue>
#include <sys/socket.h>
#include <arpa/inet.h>
#include "messages.h"

class Communicator {
    int serialfile, socket_desc, viewer_socket;
    uint16_t read_ID, send_ID, viewer_ID;
    unsigned int read_size, send_size, viewer_size, num_messages, bad_bytes, _readindex, sendskipcntr;
    bool controller_connected, server_open, viewer_connected;
    std::queue<char> instream;
    struct sockaddr_in server, client;
    const char *viewer_message;
public:
    const unsigned int &num_inbox, &num_bad_bytes;
    const bool &is_controller_connected, &is_server_open, &is_viewer_connected;
    Communicator(void);
    Communicator(uint16_t listen_ID, size_t listen_size);
    Communicator(uint16_t listen_ID, size_t listen_size, uint16_t send_ID, size_t send_size);
    ~Communicator(void);
    void set_listen_ID(uint16_t ID, size_t size);
    int read_message(char *buff); //returns how many messages there are after this one. -1 is a failure.
    void set_send_ID(uint16_t ID, size_t size);
    void send_message(const char *buff);
    void handle_messages(void);

    void start_server(int port_num, uint16_t msg_ID, size_t size, const char *msg);
    void try_connect(void);
    void broadcast_message(unsigned skip_every);
    
};
