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
    Some fluctuations are not spikes. If the true value is between two quantized increments,
        the measured value will fluctuate between them. Dequantization can solve this.
    TODO:
        Configurable anchor width
        Executable MicroFilter?
        Lowpass
        Dequantizer: Replace constantly fluctuating values with steady intermediate value
            Dequantize velocity or position signal?
        Generic Filter interface for including without knowing filter params
            calibrateMotor main creates filter, MotorCalibrator only has ptr to filter(s)
*/
#ifndef SIMPLEWALKER_PICO_FILTER_HPP
#define SIMPLEWALKER_PICO_FILTER_HPP
#include <array>


template<unsigned N>
class StaticQueue {
    static_assert(N > 1);
    unsigned idx_ {};
    std::array<float, N> data {};
public:
    static constexpr size_t SIZE = N;
    void push(float new_raw_value) {
        data.at(idx_) = new_raw_value;
        idx_ = (idx_ == (N - 1)) ? 0 : idx_ + 1;
    }
    // i 0 -> N-1 oldest -> newest
    inline float& at(unsigned i) {
        return data.at((idx_ + i) % N);
    }
    inline float oldest() {return data[idx_];}
};


template<unsigned N>
class MovingAverage {
    StaticQueue<N> queue;
    float average {};
public:
    static constexpr size_t SIZE = N;
    void new_value(float new_raw_value) {
        average -= queue.oldest();
        queue.push(new_raw_value / N);
        average += new_raw_value / N;
    }
    inline float get() const {
        return average;
    }
};


/* Removes Two-way spikes (but not one-way spikes) within a target window with size WIDTH.
 The scan window includes 1 'anchor' sample on each side of the target window.
 Extreme values outside the spread of the anchor samples are evened out with each other.
 Does not change the average value or the integral.*/
template<int WIDTH = 2>
class SymmDespiker {
    static_assert(WIDTH > 1);
    StaticQueue<WIDTH + 2> queue{};
public:
    static constexpr size_t TARGET_SIZE = WIDTH;
    static constexpr size_t SCAN_SIZE = WIDTH + 2;
    void new_value(float new_raw_value) {
        queue.push(new_raw_value);
        unsigned highest_idx = 0, lowest_idx = 0;
        for (size_t i = 1; i < SCAN_SIZE; i++) {
            if (queue.at(i) > queue.at(highest_idx)) {
                highest_idx = i;
            } else if (queue.at(i) < queue.at(lowest_idx)) {
                lowest_idx = i;
            }
        }
        unsigned higher_anchor = queue.at(0) > queue.at(WIDTH + 1) ? 0 : WIDTH + 1;
        unsigned lower_anchor  = queue.at(0) > queue.at(WIDTH + 1) ? WIDTH + 1 : 0;
        if (highest_idx == higher_anchor || lowest_idx == lower_anchor) {
            return;  // extreme value at the edge of our data window, not a spike
        } // excess would be 0 in that^ case anyway
        float excess = std::min(queue.at(highest_idx) - queue.at(higher_anchor),
                                queue.at(lower_anchor) - queue.at(lowest_idx));
        queue.at(highest_idx) -= excess;
        queue.at(lowest_idx)  += excess;
    }
    float get() {
        return queue.oldest();
    }
};


#endif  // SIMPLEWALKER_PICO_FILTER_HPP
