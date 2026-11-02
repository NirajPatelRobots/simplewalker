/* Filter functions
    Spikes: 
        Upspikes and downspikes, based on which direction they go first,
        One-way (down, then back to normal) or two-way, Two-way spikes can be symmetrical or asymmetrical
    The most common sensor error is a width=1 one-way downspike, followed by w-2 one-way downspikes.
    Down, then the same distance above normal, then normal is a width=2 two-way symmetrical downspike
        This is the shape of velocity measurements when position has a width=1 downspike.
        These are sensor errors, so we want to remove these to 0 mean.
    If the distance up and down are not the same, is a w=2 two-way asymmetrical downspike
        This is the shape of a true position shift down acompanied by a spike.
            We need to preserve the movement when we remove this spike.
    TODO:
        Stream class with filters connected to stream?
        Interpolate?
        Allow multiple despikers of various sizes, remove MovingAverage from Despiker
*/
#ifndef SIMPLEWALKER_PICO_FILTER_HPP
#define SIMPLEWALKER_PICO_FILTER_HPP
#include <array>


template<int N>
class MovingAverage {
    unsigned i {};
    std::array<float, N> data {};
    float value {};
public:
    void new_value(float new_raw_value) {
        value -= data.at(i);
        data.at(i) = new_raw_value / N;
        value += data.at(i);
        i = (i == 0) ? N - 1 : i - 1;  // iterate backwards so history(k) iterates positive => avoid % of num < 0
    }
    inline float get() const {
        return value;
    }
     // k 0 -> N-1 = most -> least recent data
    inline float history(int k) {
        return N * data.at((i + k + 1) % N);
    }
    void reset_to_average() {
        for (auto &d : data) {d = value / N;}
    }
};


/* Removes Two-way spikes (but not one-way spikes) within a target window with size WIDTH.
 The scan window includes 1 'anchor' sample on each side of the target window.
 Extreme values outside the spread of the anchor samples are evened out with each other.
 Does not change the average value or the integral.*/
template<int WIDTH = 2>
class SymmDespiker {
    MovingAverage<WIDTH + 2> averager{};
public:
    void new_value(float new_raw_value) {
        averager.new_value(new_raw_value);
        unsigned highest_idx = 0, lowest_idx = 0;
        // TODO make class members ^ so we don't have to check all of them each time ?
        for (int i = 1; i < WIDTH + 2; i++) {
            if (averager.history(i) > averager.history(highest_idx)) {
                highest_idx = i;
            } else if (averager.history(i) < averager.history(lowest_idx)) {
                lowest_idx = i;
            }
        }
        if (highest_idx == 0 || highest_idx == WIDTH + 1 || lowest_idx == 0 || lowest_idx == WIDTH + 1) {
            return;  // extreme value at the edge of our data window, not a spike
        }
        for (int i = 0; i < WIDTH + 2; i++) {
            std::cout << averager.history(i) << " ";
        }
        std::cout << " " << highest_idx << " " << lowest_idx << " Spike detected\n";
        averager.reset_to_average();
    }
    float get() {
        return averager.history(WIDTH + 1);
    }
};



#endif  // SIMPLEWALKER_PICO_FILTER_HPP
