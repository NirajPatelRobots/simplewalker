#include <gtest/gtest.h>
#include "../../microcontroller/filter/micro_filter.hpp"


const float TOLERANCE = 1e-6;
const int TEST_N = 7;


TEST(StaticQueueTest, PushAt_3) {
    const int QUEUE_SIZE = 3;
    StaticQueue<QUEUE_SIZE> queue;
    for (int i = 0; i < QUEUE_SIZE; i++) {
        EXPECT_EQ(queue.at(i), 0);
    }

    queue.push(1.0);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        const float expected_values[QUEUE_SIZE] = {0.0, 0.0, 1.0};
        EXPECT_EQ(queue.at(i), expected_values[i]);
    }

    queue.push(2.0);
    queue.push(3.0);
    for (int i = 0; i < QUEUE_SIZE; i++) {
        const float expected_values[QUEUE_SIZE] = {1.0, 2.0, 3.0};
        EXPECT_EQ(queue.at(i), expected_values[i]);
    }
}

TEST(StaticQueueTest, At_2) {
    const int QUEUE_SIZE = 2;
    StaticQueue<QUEUE_SIZE> queue;

    queue.at(1) = 1.0;
    EXPECT_EQ(queue.at(0), 0);
    EXPECT_EQ(queue.at(1), 1.0);
    queue.at(0) = 2.0;
    EXPECT_EQ(queue.at(0), 2.0);
    EXPECT_EQ(queue.at(1), 1.0);
    queue.push(3.0);
    EXPECT_EQ(queue.at(0), 1.0);
    EXPECT_EQ(queue.at(1), 3.0);
}

TEST(StaticQueueTest, Oldest_2) {
    const int QUEUE_SIZE = 2;
    StaticQueue<QUEUE_SIZE> queue;

    EXPECT_EQ(queue.oldest(), 0);
    queue.push(1.0);
    EXPECT_EQ(queue.oldest(), 0);
    queue.push(2.0);
    EXPECT_EQ(queue.oldest(), 1.0);
    queue.push(3.0);
    EXPECT_EQ(queue.oldest(), 2.0);
    queue.push(4.0);
    EXPECT_EQ(queue.oldest(), 3.0);
}


TEST(MovingAverageTest, AllSame) {
    MovingAverage<TEST_N> averager{};
    EXPECT_EQ(averager.get(), 0.0);

    const float TEST_VAL{5.2};
    for (int i = 0; i < TEST_N; i++) {
        averager.new_value(TEST_VAL);
    }
    EXPECT_NEAR(averager.get(), TEST_VAL, TOLERANCE);
}


TEST(MovingAverageTest, UpDownDownUpNormal) {
    MovingAverage<TEST_N> averager{};
    const float TEST_VAL{5.2}, SPIKE_VAL{28.4};
    for (int i = 0; i < TEST_N; i++) {
        averager.new_value(TEST_VAL);
    }
    averager.new_value(TEST_VAL + SPIKE_VAL);
    EXPECT_GT(averager.get(), TEST_VAL);
    averager.new_value(TEST_VAL - SPIKE_VAL);
    EXPECT_NEAR(averager.get(), TEST_VAL, TOLERANCE);

    averager.new_value(TEST_VAL - SPIKE_VAL);
    EXPECT_LT(averager.get(), TEST_VAL);
    averager.new_value(TEST_VAL + SPIKE_VAL);
    EXPECT_NEAR(averager.get(), TEST_VAL, TOLERANCE);
    averager.new_value(TEST_VAL);
    EXPECT_NEAR(averager.get(), TEST_VAL, TOLERANCE);
}


class SymmDespikerTest : public testing::Test {
protected:
    SymmDespiker<2> despiker{};
    const float TEST_VAL{5.2}, SPIKE_VAL{28.4}, BUMP_VAL{1.9};

    void SetUp() override {
        for (int i = 0; i < 4; i++) {
            despiker.new_value(TEST_VAL);
        }
    }
};


TEST_F(SymmDespikerTest, InitializeSymmDespiker) {
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    SymmDespiker<4> despiker_4{};
    EXPECT_EQ(despiker_4.get(), 0.0);
}


// velocity when position has a step down
TEST_F(SymmDespikerTest, Width1Downspike) {
    despiker.new_value(TEST_VAL - BUMP_VAL);
    for (int i = 0; i < 3; i++) {
        EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
        despiker.new_value(TEST_VAL);
    }
    // Downspike value comes out unchanged
    EXPECT_NEAR(despiker.get(), TEST_VAL - BUMP_VAL, TOLERANCE);
}


// velocity when position has a 1-width downspike
TEST_F(SymmDespikerTest, RemoveWidth2SymmDownSpike) {
    despiker.new_value(TEST_VAL - SPIKE_VAL);
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    despiker.new_value(TEST_VAL + SPIKE_VAL);
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);

    for (int i = 0; i < 5; i++) {
        // spikes are removed, don't affect result on way out
        despiker.new_value(TEST_VAL);
        EXPECT_NEAR(despiker.get(), TEST_VAL, 3 * TOLERANCE);  // low tolerance :/
    }
}


// velocity when position has a 1-width downspike that doesn't come all the way back up
TEST_F(SymmDespikerTest, RemoveWidth2AsymmDownSpike) {
    despiker.new_value(TEST_VAL - SPIKE_VAL);             // true change happens here, but measurement is much more negative than true value
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    despiker.new_value(TEST_VAL + SPIKE_VAL - BUMP_VAL);  // positive spike not as large as negative spike
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    despiker.new_value(TEST_VAL);                         // 2 ticks post-change, back to normal
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    despiker.new_value(TEST_VAL);                         // 3 ticks post-change, still normal
    EXPECT_NEAR(despiker.get(), TEST_VAL - BUMP_VAL, TOLERANCE);  // Spike removed, bump remains
    despiker.new_value(TEST_VAL);                         // 4 ticks post-change, still normal
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);     // Spike removed, bump is over, back to normal
    despiker.new_value(TEST_VAL);                         // 5 ticks post-change, still normal
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
}
