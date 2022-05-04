/*
Logging simplewalker data

Can output:
    header to explain other data
    relevant data of many objects
        -SensorBoss, pico communication, state, timing, state estimation
        -human readable or raw data

Can output to:
    stdout
    file
    future: desktop?
TODO:
    BUG: not always the right width
    can output to file
    Fancy macro or something so logging doesn't happen?
    smaller log class for getting data from anywhere
    Binary logger is child class?
    binary logger knows where data is
    log to desktop
*/
#ifndef SIMPLEWALKER_LOGGER
#define SIMPLEWALKER_LOGGER
#include <string>
#include <iostream>
#include <sstream>
#include "robot_state.hpp"
#include "sensorBoss.hpp"
#include <vector>
#include <chrono>

enum Logoptions {LOGNEWLINE = 0b1, LOGBINARY = 0b10};

struct Logtimes {
    float predict, correct, commreceive, log;
};

void start_logtiming(std::chrono::time_point<std::chrono::steady_clock> time); //starts timing, logtime is relative to this
void set_logtime(float &logtime); //sets this log time and starts counting from 0 again

class Logger {
    bool disable, headerSent;
    //std::ostream & ostream;
    std::string filename, header;
    std::stringstream outstr;
    unsigned options, lognum, skipcntr;
    std::vector<unsigned> endindexes;
public:
    Logger(int logoptions = 0); //default stdout
    Logger(std::string filename, int logoptions = 0);
    void log(std::string name, float x);
    void log(std::string name, const Vector3f &x);
    void log(std::string name, const RobotState &x);
    void log(const RobotState &x) {log("", x);}
    void log(std::string name, const SensorBoss &x);
    void log(const SensorBoss &x) {log("", x);}
    void log(const Logtimes &logtimes);

    bool print(unsigned skipevery = 0);
    bool dontprint(unsigned skipevery = 0); //removes logging
};

class LoadedBinaryLog {
    int len;
public:
    LoadedBinaryLog(std::string filename);
    float *datastart;
    const int &segmentlength; // how many floats in one segment (all the data saved at one call to print())
    std::vector<std::string> &log_names;
    std::vector<size_t> offsets;
};

#endif
