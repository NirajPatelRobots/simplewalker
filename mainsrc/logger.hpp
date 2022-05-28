/*
Logging simplewalker data

Can output:
    header to explain other data
    relevant data of many objects
    human readable or raw data
    Can output to stdout or file
    
TODO:
    BUG: log to file
    BUG: not always the right width
    can output to file
    Fancy macro or something so logging doesn't happen?
    smaller log class for getting data from anywhere
    Binary logger is child class?
    binary logger knows where data is
    multiple lines to text file
    settings int
    settings fieldname with no groupname
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
#include <cstdlib>
#include <fstream>
#include "rapidxml.hpp"

class WalkerSettings {
    rapidxml::xml_document<> doc;
    std::vector<char> text;
public:
    WalkerSettings(std::string filename);
    float f(const char * const fieldname) const;
    float f(const char * const groupname, const char * const fieldname) const;
    bool b(const char * const fieldname) const;
    bool b(const char * const groupname, const char * const fieldname) const;
};

struct Logtimes {
    float predict, correct, commreceive, commsend, log, sleep;
};

void start_logtiming(std::chrono::time_point<std::chrono::steady_clock> time); //starts timing, logtime is relative to this
void set_logtime(float &logtime); //sets this log time and starts counting from 0 again

class Logger {
    bool disable, headerSent, newline, savetofile;
    std::ofstream outFile;
    std::string filename, header;
    std::stringstream outstr;
    unsigned lognum, skipcntr;
    std::vector<unsigned> endindexes;
public:
    Logger(bool log_newline = false); //default stdout
    Logger(std::string filename, bool log_newline);
    ~Logger();
    void log(std::string name, float x);
    void log(std::string name, const Vector3f &x);
    void log(std::string name, const RobotState &x);
    void log(const RobotState &x) {log("", x);}
    void log(std::string name, const SensorBoss &x);
    void log(const SensorBoss &x) {log("", x);}
    void log(const Logtimes &logtimes);
    void log(std::string name, const WalkerSettings &settings);
    void log(std::string groupname, std::string name, const WalkerSettings &settings);

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
