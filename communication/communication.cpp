#include "communication.hpp"


bool Communicator::id_is_registered(int16_t query_id) const {
    for (auto inbox : inboxes) {if (inbox->msgID == query_id) return true;} return false;
}

MessageBoxInterface *Communicator::get_inbox(int16_t query_id) const {
    for (auto inbox : inboxes) {if (inbox->msgID == query_id) return inbox;} return nullptr;
}

void Communicator::clear_all_messages() {
    for (auto inbox : inboxes) inbox->clear();
}

bool Communicator::add_inbox(MessageBoxInterface *new_inbox) {
    if (id_is_registered(new_inbox->msgID)) {return false;}
    inboxes.push_back(new_inbox); return true;
}

void Communicator::receive_messages() {
    if (!receive_bytes()) {
        return;
    }
    MessageBoxInterface *inbox = parse_buffer_inbox();
    while (inbox && instream.size() >= inbox->msg_len) {
        inbox->set_message(instream.begin(), instream.begin() + inbox->msg_len);
        for (unsigned i = 0; i < inbox->msg_len; i++) {
            instream.pop_front();
        }
        inbox = parse_buffer_inbox();
    }
}

MessageBoxInterface* Communicator::parse_buffer_inbox() {
    if (instream.size() < 2) return nullptr;
    uint16_t ID = ((uint8_t)instream.at(0) + ((uint16_t)((uint8_t)instream.at(1)) << 8));
    MessageBoxInterface* inbox = get_inbox(ID);
    if (!inbox) {
        unexpected_bytes_in.push_back(instream.at(0));
        instream.pop_front();
    }
    return inbox;
}

void Communicator::flush_message_queue(int num_to_discard, bool print_wait_message) {
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

void Communicator::print_unexpected_bytes(bool do_ascii) {
    static size_t last_num_bad_bytes{0};
    if (unexpected_bytes_in.size() != last_num_bad_bytes) {
        last_num_bad_bytes = unexpected_bytes_in.size();
        std::cout << "Controller bad bytes in: " <<  unexpected_bytes_in.size() << std::hex;
        for (uint8_t &bad_byte : unexpected_bytes_in) std::cout << " " << bad_byte;
        if (do_ascii)
            for (uint8_t &bad_byte : unexpected_bytes_in) std::cout << " " << (char)(bad_byte);
        std::cout << std::dec << std::endl;
    }
}
