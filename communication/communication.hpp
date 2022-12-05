/* generic communication for simplewalker 
MessageBox are boxes that hold generic messages, to be used by generic communicators. Subclassed into specific messages.
Communicator is interface class, subclassed into different communications methods
October 2022
TODO:
*/
#include <deque>
#include <string>
#include <any>

#include <iostream>

class Communicator;

class MessageBoxInterface {
protected:
    Communicator &comm;
public:
    MessageBoxInterface(int16_t _msgID, size_t _msg_len, Communicator &communicator) :
        comm(communicator), msgID(_msgID), msg_len(_msg_len) {}
    const int16_t msgID;
    const size_t msg_len;
    virtual void set_message(const std::any &in_data) = 0;
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
    std::string &name;
    explicit Communicator(std::string &_name)
            : num_bad_bytes_in_(0), num_bad_bytes_out_(0), inboxes(), name_(_name),
              num_bad_bytes_in(num_bad_bytes_in_), num_bad_bytes_out(num_bad_bytes_out_), name(name_) {}
    void add_inbox(MessageBoxInterface *new_inbox) {
        if (id_is_registered(new_inbox->msgID)) {throw std::invalid_argument("message ID already registered");}
        inboxes.push_back(new_inbox);
    }
    const std::deque<MessageBoxInterface *> &get_inboxes() const {return inboxes;}
    bool id_is_registered(int query_id) {
        for (auto inbox : inboxes) {if (inbox->msgID == query_id) return true;} return false;
    }
    virtual void receive_messages() = 0;
    virtual int send(const MessageBoxInterface &outbox, std::any message) = 0; //return 0 on success
    virtual ~Communicator() = default;
};


template <typename T>
class MessageInbox : public MessageBoxInterface {
protected:
    std::deque<T> messages;
public:
    MessageInbox(int16_t _msgID, Communicator &communicator)
        : MessageBoxInterface(_msgID, sizeof(T), communicator), messages()  {comm.add_inbox(this);}
    // sets message_out if a message is available. Returns the number of messages available after the set, return < 0 is failure
    int get_newest(T &message_out) {
        if (num_available() > 0) {
            message_out = messages.back();
            messages.pop_back();
            return num_available();
        }
        return -1;
    }
    // receive a new message. usually be called by communicator. can throw std::bad_any_cast
    void set_message(const std::any &in_data) override {
        messages.push_back(std::any_cast<T>(in_data));  // TODO: catch std::bad_any_cast
    }
    void clear() {while (!messages.empty()) messages.pop_back();}
    int num_available() {return messages.size();}
    ~MessageInbox() override = default;
};


template <typename T>
class MessageOutbox : public MessageBoxInterface {
public:
    T message;
    MessageOutbox(const int16_t _msgID, Communicator &communicator)
            : MessageBoxInterface(_msgID, sizeof(T), communicator), message({}) {}
    int send() {return comm.send(*this, message);}
    // set_message is included for inheritance, but better to just directly set MessageOutbox.message
    void set_message(const std::any &in_data) override {
        message = std::any_cast<T>(in_data);  // TODO: catch std::bad_any_cast
    }
    ~MessageOutbox() override = default;
};


class LoopbackCommunicator : public Communicator {
public:
    explicit LoopbackCommunicator(std::string name) : Communicator(name) {}
    int send(const MessageBoxInterface &outbox, std::any message) override {
        for (auto inbox : inboxes) if (inbox->msgID == outbox.msgID) {
            inbox->set_message(message);
        }
        return 0;
    }
    void receive_messages() override {}
    ~LoopbackCommunicator() override = default;
};

