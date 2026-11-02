#include <gtest/gtest.h>
#include "../../microcontroller/filter/micro_filter.hpp"


const float TOLERANCE = 1e-6;
const int TEST_N = 7;


TEST(MovingAverageTest, AllSame) {
    MovingAverage<TEST_N> averager{};
    EXPECT_EQ(averager.get(), 0.0);

    const float TEST_VAL{5.2};
    for (int i = 0; i < TEST_N; i++) {
        averager.new_value(TEST_VAL);
        for (int j = 0; j <= i; j++) {
            EXPECT_EQ(averager.history(j), TEST_VAL);
        }
        for (int j = i + 1; j < TEST_N; j++) {
            EXPECT_EQ(averager.history(j), 0.0);
        }
    }
    EXPECT_NEAR(averager.get(), TEST_VAL, TOLERANCE);
}


TEST(MovingAverageTest, UpSpikeDownSpike) {
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
    
    EXPECT_EQ(averager.history(0), TEST_VAL);
    EXPECT_EQ(averager.history(1), TEST_VAL + SPIKE_VAL);
    EXPECT_EQ(averager.history(2), TEST_VAL - SPIKE_VAL);
    EXPECT_EQ(averager.history(3), TEST_VAL - SPIKE_VAL);
    EXPECT_EQ(averager.history(4), TEST_VAL + SPIKE_VAL);
    EXPECT_EQ(averager.history(5), TEST_VAL);
    
    averager.reset_to_average();
    for (int i = 0; i < TEST_N; i++) {
        EXPECT_NEAR(averager.history(i), TEST_VAL, TOLERANCE);
    }
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
    float total_of_next_4_values = despiker.get();
    for (int i = 0; i < 3; i++) {
        // std::cout << i << " = test loop to 3\n";
        EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
        despiker.new_value(TEST_VAL);
        total_of_next_4_values += despiker.get();
    }
    // Downspike value comes out unchanged
    EXPECT_NEAR(despiker.get(), TEST_VAL - BUMP_VAL, TOLERANCE);
    EXPECT_NEAR(total_of_next_4_values, TEST_VAL * 4 - BUMP_VAL, TOLERANCE);  // integrated change = total position change, calculus is not broken
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
        EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    }
}


// velocity when position has a 1-width downspike that doesn't come all the way back up
TEST_F(SymmDespikerTest, RemoveWidth2AsymmDownSpike) {
    despiker.new_value(TEST_VAL - SPIKE_VAL);             // true change happens here, but measurement is much more negative than true value
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    despiker.new_value(TEST_VAL + SPIKE_VAL - BUMP_VAL);  // positive spike not as large as negative spike
    EXPECT_NEAR(despiker.get(), TEST_VAL, TOLERANCE);
    float total_of_next_4_values = 0;
    for (int i = 0; i < 4; i++) {
        despiker.new_value(TEST_VAL);                     // 2 + i ticks later, back to normal. => Reaction time = WIDTH
        EXPECT_NEAR(despiker.get(), TEST_VAL - BUMP_VAL / 4, TOLERANCE); // bump is spread out over ticks 2 - 5
        total_of_next_4_values += despiker.get();
    }
    // integrated change = total position change, calculus is not broken
    EXPECT_NEAR(total_of_next_4_values, TEST_VAL * 4 - BUMP_VAL, 2 * TOLERANCE);  // Why double tolerance?
}
