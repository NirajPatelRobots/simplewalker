#include "comm_tcp.hpp"
#include <sys/socket.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <pthread.h>

//thank you to https://github.com/Mad-Scientist-Monkey/sockets-ccpp-rpi


const size_t BUF_SIZE{256};
const int SO_REUSEADDR_VALUE {1};
socklen_t sock_length = sizeof(struct sockaddr_in);
void *connection_handler(void *state_void);

struct TCPCommunicatorState {
    struct sockaddr_in server{}, client{};
    int file_desc{}, socket_desc{};
    pthread_t connection_thread{};
    deque<char> instream_buffer{};
    void disconnect() {
        socket_desc = 0;
    }
};

TCPCommunicator::TCPCommunicator(const string &_name)
    : Communicator(_name), state(std::make_unique<TCPCommunicatorState>())
    {}

TCPCommunicator::~TCPCommunicator() {
    stop_server();
    pthread_join(state->connection_thread, nullptr);
}

void TCPCommunicator::start_server(int port_num) {
    if (is_connected()) return;
    state->file_desc = socket(AF_INET , SOCK_STREAM , IPPROTO_IP);
    if (state->file_desc == -1) {
        std::cerr<< "Could not create socket" << std::endl;
        return;
    }
    setsockopt(state->file_desc, SOL_SOCKET, SO_REUSEADDR, &SO_REUSEADDR_VALUE, sizeof(int)); // allow reconnection
    state->server.sin_family = AF_INET;
    state->server.sin_addr.s_addr = INADDR_ANY;
    state->server.sin_port = htons( port_num );
    if( bind(state->file_desc, (struct sockaddr *)&(state->server), sizeof(state->server)) < 0) {
        std::cerr << "Server bind failed: " << state->file_desc << " errno: " << errno << "\n";
        return;
    }
    std::cout<< "Waiting for TCP connections...\n";
    if(pthread_create(&state->connection_thread, nullptr, connection_handler, (void *)state.get()) != 0) {
        std::cerr<<"couldn't create thread\n"; return;
    }
}

void *connection_handler(void *state_void) {
    auto *state = (TCPCommunicatorState*)state_void;
    listen(state->file_desc, 3);
    while ((state->socket_desc = accept(state->file_desc, (struct sockaddr *) &state->client, &sock_length)) >= 0) {
        std::cout << "Connected to " << inet_ntoa(state->client.sin_addr) << " port: " << ntohs(state->client.sin_port)
                  << " socket: " << state->socket_desc << "\n";
        size_t read_size;
        char buf[BUF_SIZE];
        while (true) {
            if ((read_size = recv(state->socket_desc, buf, BUF_SIZE, 0)) <= 0) {
                if (read_size == 0) {
                    std::cerr << "Client Disconnected\n";
                    state->disconnect();
                    break;
                } else {
                    std::cerr << "TCP Read err " << read_size << "\n";
                }
            }
            // instream_buffer mutex
            for (size_t i = 0; i < read_size; i++) {
                printf("%c", buf[i]);  // TODO remove
                state->instream_buffer.push_back(buf[i]);
            }
            // instream_buffer mutex
        }
    }
    std::cerr << "Couldn't accept connection: " << errno << "\n";
    return nullptr;
}


bool TCPCommunicator::is_connected() const {
    return (state->socket_desc > 0);
}

void TCPCommunicator::stop_server() {
    close(state->file_desc);
    state->disconnect();
}

int TCPCommunicator::send(const MessageBoxInterface &outbox, const char *data_start) {
    if (is_connected()) {
        if (write(state->socket_desc, data_start, outbox.msg_len) == (ssize_t)outbox.msg_len)
            return 0;
        std::cerr << "TCP Send: Wrong number of bytes written\n";
        return 1;
    }
    return 2;
}

bool TCPCommunicator::receive_bytes() {
    if (state->instream_buffer.empty()) return false;
    // instream_buffer mutex
    if (instream.empty()) {
        instream = std::move(state->instream_buffer);
    } else {
        instream.insert(instream.end(), state->instream_buffer.begin(), state->instream_buffer.end());
    }
    state->instream_buffer.clear();
    // instream_buffer mutex
    return true;
}

