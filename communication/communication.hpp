/* generic communication for simplewalker 
MessageBox are boxes that hold generic messages, to be used by generic communicators. Subclassed into specific messages.
Communicator is interface class, subclassed into different communications methods
October 2022
TODO:
    there's gotta be a better (safer?) way to do this char setting stuff
        char* for data_start in MessageBoxInterface?
    Maybe split MessageBoxInterface into interface classes for inbox and outbox
    decide between msg_len and SIZE
    detect late message
    custom exception
    skip every so many sends
*/
#ifndef SIMPLEWALKER_COMM_HPP
#define SIMPLEWALKER_COMM_HPP

#include <deque>
#include <string>
#include <iostream>
using std::deque, std::string, std::to_string;

class Communicator;

class MessageBoxInterface {
public:
    Communicator &comm;
    MessageBoxInterface(int16_t _msgID, size_t _msg_len, Communicator &communicator) :
        comm(communicator), msgID(_msgID), msg_len(_msg_len) {}
    const int16_t msgID;
    const size_t msg_len;
    virtual void set_message(deque<char>::iterator data_start, deque<char>::iterator data_end) = 0;
    virtual void clear() = 0;
    virtual int num_available() { return 0;}
    virtual ~MessageBoxInterface() = default;
};


// Communicator interface class works with MessageBoxes, subclasses are implementations of communication
class Communicator {
protected:
    unsigned int num_bad_bytes_in_, num_bad_bytes_out_;
    std::deque<MessageBoxInterface *> inboxes;
    std::string name_;
public:
    const unsigned int &num_bad_bytes_in, &num_bad_bytes_out;
    const std::string &name;
    explicit Communicator(std::string &_name)
            : num_bad_bytes_in_(0), num_bad_bytes_out_(0), inboxes(), name_(_name),
              num_bad_bytes_in(num_bad_bytes_in_), num_bad_bytes_out(num_bad_bytes_out_), name(name_) {}

    void add_inbox(MessageBoxInterface *new_inbox) {
        if (id_is_registered(new_inbox->msgID)) {throw std::invalid_argument("message ID already registered");}
        inboxes.push_back(new_inbox);
    }
    const deque<MessageBoxInterface *> &get_inboxes() const {return inboxes;}
    bool id_is_registered(int query_id) {
        for (auto inbox : inboxes) {if (inbox->msgID == query_id) return true;} return false;
    }
    MessageBoxInterface *get_inbox(int query_id) {
        for (auto inbox : inboxes) {if (inbox->msgID == query_id) return inbox;} return nullptr;
    }

    virtual void receive_messages() = 0;
    virtual int send(const MessageBoxInterface &outbox, const char *data_start) = 0; //return 0 on success
    void clear_all_messages() {
        for (auto inbox : inboxes) inbox->clear();
    }
    void flush_message_queue(int num_to_discard = 3, bool print_wait_message = false) {
        if (print_wait_message)
            std::cout<<"Waiting for " << name_ << " messages...\r" << std::flush;
        int messages_received = 0;
        clear_all_messages();
        while (messages_received < num_to_discard) {
            receive_messages();
            for (auto inbox : inboxes) {
                messages_received += inbox->num_available();
            }
            clear_all_messages();
        }
        if (print_wait_message)
            std::cout<<"                                                         \r" << std::flush;
    }
    virtual ~Communicator() = default;
};


template <typename T>
class MessageInbox : public MessageBoxInterface {
protected:
    deque<T> messages;
public:
    MessageInbox(int16_t _msgID, Communicator &communicator)
        : MessageBoxInterface(_msgID, SIZE, communicator), messages()  {comm.add_inbox(this);}
    /* sets message_out if a message is available.
    Returns the number of messages available after the set, return < 0 is failure */
    int get_newest(T &message_out) {
        if (num_available() > 0) {
            message_out = messages.back();
            messages.pop_back();
            return num_available();
        }
        return -1;
    }
    // receive a new message. usually be called by communicator. can throw std::logic_error
    void set_message(deque<char>::iterator data_start, deque<char>::iterator data_end) override {
        if (data_end - data_start != SIZE)
            throw std::logic_error("INBOX ERROR: size=" + to_string(SIZE) + " data size=" + to_string(data_end - data_start));
        messages.emplace_back();
        for (int i = 0; i<SIZE; i++) {
            ((char*)&messages.back())[i] = *(data_start + i);
        }
    }
    void set(const T& message) {
        messages.push_back(message);
    }
    void clear() override {while (!messages.empty()) messages.pop_back();}
    int num_available() override {return messages.size();}
    ~MessageInbox() override = default;
    inline const static int SIZE = sizeof(T) / sizeof(char);
};


template <typename T>
class MessageOutbox : public MessageBoxInterface {
public:
    T message;
    MessageOutbox(const int16_t _msgID, Communicator &communicator)
            : MessageBoxInterface(_msgID, SIZE, communicator), message({}) {}
    int send() {return comm.send(*this, (char*)&message);}
    // set_message is included for inheritance, but better to just directly set MessageOutbox.message
    void set_message(deque<char>::iterator data_start, deque<char>::iterator data_end) override {
        if (data_end - data_start != SIZE)
            throw std::logic_error("OUTBOX ERROR: size=" + to_string(SIZE) + " data size=" + to_string(data_end - data_start));
        for (int i = 0; i<SIZE; i++) {
            ((char*)&message)[i] = *(data_start + i);
        }
    }
    void clear() override {message = T();}
    ~MessageOutbox() override = default;
    inline const static int SIZE = sizeof(T) / sizeof(char);
};


class LoopbackCommunicator : public Communicator {
public:
    explicit LoopbackCommunicator(std::string name) : Communicator(name) {}
    int send(const MessageBoxInterface &outbox, const char *data_start) override {
        deque<char> data{};
        for (size_t i=0; i < outbox.msg_len; i++)
            data.push_back(*(data_start+i));
        MessageBoxInterface *inbox = get_inbox(outbox.msgID);
        if (!inbox) return 1;
        if (inbox->msg_len == outbox.msg_len) {
            inbox->set_message(data.begin(), data.end());
        }
        return 0;  // success
    }
    void receive_messages() override {}
    ~LoopbackCommunicator() override = default;
};

#endif
