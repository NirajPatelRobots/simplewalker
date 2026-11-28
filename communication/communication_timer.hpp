/* TODO:
 *     test?
 */
#ifndef SIMPLEWALKER_COMM_TIMER_HPP
#define SIMPLEWALKER_COMM_TIMER_HPP
#include <chrono>
#include <thread>
#include "communication.hpp"

namespace chrono = std::chrono;
typedef chrono::time_point<chrono::steady_clock> time_point;

chrono::microseconds MAX_LISTEN_INTERVAL {20000};  // guess


template< class Rep, class Period >
long count_us(chrono::duration<Rep, Period> duration) {
    return chrono::duration_cast<std::chrono::microseconds>(duration).count();
}


template<typename T, unsigned skip_every>
int64_t listen_timeout(MessageInbox<T, skip_every> &inbox, int64_t wait_us,
                        chrono::microseconds increment_us = MAX_LISTEN_INTERVAL) {
    const time_point start_point = chrono::steady_clock::now();
    time_point sleep_target = start_point;
    chrono::microseconds wait_duration{wait_us};
    while ((sleep_target < start_point + wait_duration) && (inbox.num_available() == 0)) {
        sleep_target = std::min(start_point + wait_duration, sleep_target + increment_us);
        std::this_thread::sleep_until(sleep_target);
        inbox.comm.receive_messages();
    }
    return count_us(sleep_target - start_point);
}


void listen_duration(Communicator &comm, chrono::microseconds wait_us,
                        time_point start_point = chrono::steady_clock::now()) {
    auto sleep_target = start_point;
    while (sleep_target < start_point + wait_us) {
        sleep_target = std::min(start_point + wait_us, sleep_target + MAX_LISTEN_INTERVAL);
        std::this_thread::sleep_until(sleep_target);
        comm.receive_messages();
    }
}

void listen_duration(Communicator &comm, unsigned wait_us,
                        time_point start_point = chrono::steady_clock::now()) {
    listen_duration(comm, chrono::microseconds(wait_us), start_point);
}


class CommTimer {
    time_point timestart;
    time_point loopstart;
    time_point missing_readtime;
public:
    chrono::microseconds timestep;
    chrono::microseconds msgwaittime;
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
        listen_duration(inbox.comm, timestep, loopstart);

        loopstart = loopstart + timestep;
        bool waiting_for_late = false;
        time_point late_readtime;
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
    const time_point &last_missing_msg_time() { return missing_readtime; };
    const time_point &initial_timestamp() { return timestart; };
    const time_point &loop_timestamp() { return loopstart; };
    auto duration() const { return loopstart - timestart; };
};

#endif // SIMPLEWALKER_COMM_TIMER_HPP
