/* TODO:
 *     test?
 */
#ifndef SIMPLEWALKER_COMM_TIMER_HPP
#define SIMPLEWALKER_COMM_TIMER_HPP
#include <chrono>
#include <thread>
#include "communication.hpp"

namespace chrono = std::chrono;

template< class Rep, class Period >
long count_us(chrono::duration<Rep, Period> duration) {
    return chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

class CommTimer {
    chrono::time_point<chrono::steady_clock> timestart;
    chrono::time_point<chrono::steady_clock> loopstart;
    chrono::time_point<chrono::steady_clock> missing_readtime;
public:
    chrono::microseconds timestep;
    chrono::microseconds msgwaittime;
    chrono::microseconds max_listen_interval {20000};  // guess
    chrono::microseconds acceptable_delay;
    bool print_stdout;
    CommTimer(unsigned timestep_us, unsigned max_wait_time, unsigned acceptable_delay_us = 0, unsigned start_delay_us = 0) :
            timestart(chrono::steady_clock::now() + chrono::microseconds(timestep_us + start_delay_us)),
            loopstart(timestart),
            missing_readtime(),
            timestep(timestep_us),
            msgwaittime(max_wait_time),
            acceptable_delay(acceptable_delay_us),
            print_stdout(true)
    {}

    template<typename T, unsigned skip_every>
    void wait_receive_message(MessageInbox<T, skip_every> &inbox, T &message) {
        if (inbox.num_available()) {
            if (inbox.num_available() > 1 && print_stdout)
                std::cout << "Clearing " << inbox.num_available() << " messages stored before wait\n";
            inbox.clear();
        }
        bool message_was_missing = message_is_missing();
        sleep_until_next_loop(inbox.comm);
        bool waiting_for_late = false;
        chrono::time_point<chrono::steady_clock> late_readtime;
        while (inbox.get_newest(message) < 0) {
            inbox.comm.receive_messages();
            late_readtime = chrono::steady_clock::now();
            waiting_for_late = true;
            if (late_readtime - loopstart > msgwaittime) {
                missing_readtime = late_readtime;
                if (print_stdout && !message_was_missing)
                    std::cout << "ERROR: message missing\n";
                break;
            }
        }
        if (waiting_for_late) {
            if (!message_is_missing()) {
                auto delay = count_us(late_readtime - loopstart);
                static long total_delay = 0;
                total_delay += delay;
                if (print_stdout && !message_is_missing() && late_readtime - loopstart > acceptable_delay) {
                    std::cout << "\nWARNING: message " << inbox.msgID << " is " << delay << " us late, ratio: "
                              << (float)total_delay / (float)count_us(late_readtime - timestart) << "\n";
                }
            }
            loopstart = late_readtime; // slow down time so we don't get ahead
        }
    }

    inline bool message_is_missing() { return missing_readtime + timestep == loopstart; }
    const chrono::time_point<chrono::steady_clock> &last_missing_msg_time() { return missing_readtime; };
    const chrono::time_point<chrono::steady_clock> &initial_timestamp() { return timestart; };
    const chrono::time_point<chrono::steady_clock> &loop_timestamp() { return loopstart; };
    auto duration() const { return loopstart - timestart; };
protected:
    void sleep_until_next_loop(Communicator &comm) {
        auto sleep_target = loopstart;
        while (sleep_target < loopstart + timestep) {
            sleep_target = std::min(loopstart + timestep, sleep_target + max_listen_interval);
            std::this_thread::sleep_until(sleep_target);
            comm.receive_messages();
        }
        loopstart = sleep_target;  // = loopstart + timestep
    }
};

#endif // SIMPLEWALKER_COMM_TIMER_HPP
