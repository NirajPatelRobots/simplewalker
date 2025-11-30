/* test generic communication for simplewalker
November 2022
TODO:
    define an equality comparator for test_struct_1
    mock communicator?
*/
#include <memory>
#include <any>
#include <gtest/gtest.h>
#include "communication.hpp"

struct test_struct_1 {
    float dynamic_feedback;
    int num_animals;
    bool has_electricity;
};
const int test_ID_1 = 140;
const int size_1 = sizeof(test_struct_1) / sizeof(char);

struct test_struct_2 {
    float static_feedback;
    int false_readings;
    bool is_luminous;
};
const int test_ID_2 = 3000;

class MockCommunicator : public Communicator {
public:
    explicit MockCommunicator(std::string name) : Communicator(name) {}
    ~MockCommunicator() override = default;
    //MOCK_METHOD(int, send, (const MessageBoxInterface &outbox, std::any message), (override));
};

class BasicCommunicationTest : public ::testing::Test {
protected:
    std::unique_ptr<LoopbackCommunicator> loopback_comm;
    test_struct_1 received_message;
    void SetUp() override {
        loopback_comm = std::make_unique<LoopbackCommunicator>("test_communicator");
        received_message = {0, 0, false};
    }
};


TEST_F(BasicCommunicationTest, CreateInbox) {
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    EXPECT_EQ(test_ID_1, test_inbox_1.msgID);
    EXPECT_EQ(test_ID_1, loopback_comm->get_inboxes().at(0)->msgID);
    EXPECT_EQ(sizeof(test_struct_1), test_inbox_1.msg_len);
    EXPECT_EQ(sizeof(test_struct_1), loopback_comm->get_inboxes().at(0)->msg_len);
    EXPECT_EQ(test_inbox_1.msg_len, test_inbox_1.SIZE);
}

TEST_F(BasicCommunicationTest, InboxIsEmpty) {
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    EXPECT_EQ(0, test_inbox_1.num_available());
    EXPECT_EQ(-1, test_inbox_1.get_newest(received_message));

    EXPECT_EQ(0.0, received_message.dynamic_feedback);
    EXPECT_EQ(0, received_message.num_animals);
    EXPECT_FALSE(received_message.has_electricity);
    EXPECT_EQ(0, test_inbox_1.num_available());
}

TEST_F(BasicCommunicationTest, NoMultipleInboxSameID) {
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    EXPECT_THROW({
        MessageInbox<test_struct_1> test_inbox_2(test_ID_1, *loopback_comm);
    }, std::invalid_argument);
    EXPECT_THROW({
        MessageInbox<test_struct_2> test_inbox_2(test_ID_1, *loopback_comm);
    }, std::invalid_argument);
}

TEST_F(BasicCommunicationTest, InboxReceiveMessage) {
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    test_struct_1 test_message{.dynamic_feedback = 4.5, .num_animals = 8, .has_electricity=true};
    deque<char> test_data{};
    for (size_t i=0; i < size_1; i++)
        test_data.push_back(*(((char*)&test_message) + i));
    test_inbox_1.set_message(test_data.begin(), test_data.end());
    ASSERT_EQ(1, test_inbox_1.num_available());
    EXPECT_EQ(0, test_inbox_1.get_newest(received_message));

    EXPECT_EQ(test_message.dynamic_feedback, received_message.dynamic_feedback);
    EXPECT_EQ(test_message.num_animals, received_message.num_animals);
    EXPECT_EQ(test_message.has_electricity, received_message.has_electricity);
    EXPECT_EQ(0, test_inbox_1.num_available());
}

TEST_F(BasicCommunicationTest, InboxReceiveMultipleMessages) {
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    test_struct_1 test_message{.dynamic_feedback = 4.5, .num_animals = 8, .has_electricity=true};
    test_inbox_1.set(test_struct_1{});   // first, empty message
    test_inbox_1.set(test_message);
    ASSERT_EQ(2, test_inbox_1.num_available());

    EXPECT_EQ(1, test_inbox_1.get_newest(received_message));
    EXPECT_EQ(test_message.dynamic_feedback, received_message.dynamic_feedback);
    EXPECT_EQ(test_message.num_animals, received_message.num_animals);
    EXPECT_EQ(test_message.has_electricity, received_message.has_electricity);

    EXPECT_EQ(0, test_inbox_1.get_newest(received_message));
    EXPECT_EQ(0.0, received_message.dynamic_feedback);
    EXPECT_EQ(0, received_message.num_animals);
    EXPECT_FALSE(received_message.has_electricity);
}

TEST_F(BasicCommunicationTest, InboxClear) {
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    test_struct_1 test_message{.dynamic_feedback = 4.5, .num_animals = 8, .has_electricity=true};
    test_inbox_1.set(test_struct_1{});
    test_inbox_1.set(test_message);
    ASSERT_EQ(2, test_inbox_1.num_available());
    test_inbox_1.clear();
    EXPECT_EQ(0, test_inbox_1.num_available());
}

TEST_F(BasicCommunicationTest, CreateOutbox) {
    MessageOutbox<test_struct_1> test_outbox_1(test_ID_1, *loopback_comm);
    EXPECT_EQ(0.0, test_outbox_1.message.dynamic_feedback);
    EXPECT_EQ(0, test_outbox_1.message.num_animals);
    EXPECT_FALSE(test_outbox_1.message.has_electricity);
    test_outbox_1.message.dynamic_feedback = 1000.23;
    EXPECT_EQ(0, loopback_comm->get_inboxes().size());
    EXPECT_EQ(sizeof(test_struct_1), test_outbox_1.msg_len);
    EXPECT_EQ(test_ID_1, test_outbox_1.msgID);
}

TEST_F(BasicCommunicationTest, SendLoopback) {
    MessageInbox<test_struct_2> test_inbox_2(test_ID_2, *loopback_comm);   // should ignore this
    MessageInbox<test_struct_1> test_inbox_1(test_ID_1, *loopback_comm);
    MessageOutbox<test_struct_1> test_outbox_1(test_ID_1, *loopback_comm);
    EXPECT_EQ(2, loopback_comm->get_inboxes().size());
    test_outbox_1.message = {.dynamic_feedback = 1000.23, .num_animals = 3, .has_electricity=true};

    ASSERT_EQ(0, test_outbox_1.send());
    loopback_comm->receive_messages();
    ASSERT_EQ(1, test_inbox_1.num_available());
    EXPECT_EQ(0, test_inbox_2.num_available());
    EXPECT_EQ(0, test_inbox_1.get_newest(received_message));

    EXPECT_EQ(test_outbox_1.message.dynamic_feedback, received_message.dynamic_feedback);
    EXPECT_EQ(test_outbox_1.message.num_animals, received_message.num_animals);
    EXPECT_EQ(test_outbox_1.message.has_electricity, received_message.has_electricity);
}

TEST_F(BasicCommunicationTest, ForwardMessage) {
    std::unique_ptr<LoopbackCommunicator> other_comm;
    other_comm = std::make_unique<LoopbackCommunicator>("other_comm");

    // outbox, message to send, inboxes to receive
    MessageOutbox<test_struct_1> test_outbox(test_ID_1, *loopback_comm);
    test_outbox.message = {.dynamic_feedback = 1000.23, .num_animals = 3, .has_electricity=true};
    // loopback_comm forwards message id 1 to other_comm
    MessageInbox<test_struct_1> test_inbox(test_ID_1, *loopback_comm, other_comm.get());
    MessageInbox<test_struct_1> other_inbox(test_ID_1, *other_comm);

    // send message 1
    ASSERT_EQ(0, test_outbox.send());
    loopback_comm->receive_messages();

    // check it's received by loopback
    ASSERT_EQ(1, test_inbox.num_available());
    EXPECT_EQ(0, test_inbox.get_newest(received_message));
    EXPECT_EQ(test_outbox.message.dynamic_feedback, received_message.dynamic_feedback);
    EXPECT_EQ(test_outbox.message.num_animals, received_message.num_animals);
    EXPECT_EQ(test_outbox.message.has_electricity, received_message.has_electricity);

    // check it's received by other_comm
    received_message = {0, 0, false};
    ASSERT_EQ(1, other_inbox.num_available());
    EXPECT_EQ(0, other_inbox.get_newest(received_message));
    EXPECT_EQ(test_outbox.message.dynamic_feedback, received_message.dynamic_feedback);
    EXPECT_EQ(test_outbox.message.num_animals, received_message.num_animals);
    EXPECT_EQ(test_outbox.message.has_electricity, received_message.has_electricity);
}

TEST_F(BasicCommunicationTest, DontForwardMessage) {
    std::unique_ptr<LoopbackCommunicator> other_comm;
    other_comm = std::make_unique<LoopbackCommunicator>("other_comm");

    // outbox, message to send, inboxes to receive
    MessageOutbox<test_struct_1> test_outbox(test_ID_1, *loopback_comm);
    test_outbox.message = {.dynamic_feedback = 1000.23, .num_animals = 3, .has_electricity=true};
    MessageInbox<test_struct_1> test_inbox(test_ID_1, *loopback_comm);
    MessageInbox<test_struct_1> other_inbox(test_ID_1, *other_comm);
    // loopback_comm forwards message id 2, NOT ID 1
    MessageInbox<test_struct_1> test_inbox_2(test_ID_2, *loopback_comm, other_comm.get());

    // send message 1
    ASSERT_EQ(0, test_outbox.send());
    loopback_comm->receive_messages();

    // check it's received by loopback
    ASSERT_EQ(1, test_inbox.num_available());
    EXPECT_EQ(0, test_inbox.get_newest(received_message));

    // check it's NOT received by other_comm
    ASSERT_EQ(0, other_inbox.num_available());
    EXPECT_EQ(-1, other_inbox.get_newest(received_message));
}

TEST_F(BasicCommunicationTest, CantForwardToSelf) {
    EXPECT_THROW({
        MessageInbox<test_struct_1> test_inbox(test_ID_1, *loopback_comm, loopback_comm.get());
    }, std::invalid_argument);
}
