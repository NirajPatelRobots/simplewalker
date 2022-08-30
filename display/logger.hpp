/*
Logging simplewalker data

Can output:
    header to explain other data
    relevant data of many objects
    human readable or raw data
    Can output to stdout or file
    
TODO:
    BUG: log to file causes crash
    BUG: not always the right width
    Fancy macro or something so logging doesn't happen?
    Binary logger is child class?
    binary logger knows where data is
    multiple lines to text file
    settings fieldname with no groupname
    add/remove prefix from logger name for surrounding code blocks so we can tell logs come from that block
*/
#ifndef SIMPLEWALKER_LOGGER
#define SIMPLEWALKER_LOGGER
#include <sstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include "walkerUtils.hpp"
#include "rapidxml.hpp"


class WalkerSettings {
    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> *settings_node = NULL;
    std::vector<char> text;
public:
    WalkerSettings(string filename);
    const char *cstr(const char * const fieldname) const;
    const char *cstr(const char * const groupname, const char * const fieldname) const;
    float f(const char * const fieldname) const;
    float f(const char * const groupname, const char * const fieldname) const;
    int i(const char * const fieldname) const;
    int i(const char * const groupname, const char * const fieldname) const;
    bool b(const char * const fieldname) const;
    bool b(const char * const groupname, const char * const fieldname) const;
};


struct Logtimes {
    float predict, correct, commreceive, commsend, log, sleep;
};

void start_logtiming(std::chrono::time_point<std::chrono::steady_clock> time); //starts timing, logtime is relative to this
void set_logtime(float &logtime); //sets this log time and starts counting from 0 again

class Logger {
protected:
    bool disable, headerSent, newline, savetofile;
    std::ofstream outFile;
    std::string filename, header;
    std::stringstream outstr;
    unsigned lognum, skipcntr;
    std::vector<unsigned> fieldEndIndexes;
    void make_field(string name);
    void print_line(void);
public:
    Logger(string filename = "", bool log_newline = false);  //default file is stdout
    ~Logger();
    void log(string name, float x);
    void log(string name, const Eigen::Ref<const Eigen::MatrixXf> &x);
    void log(const Logtimes &logtimes);
    void log(string name, const WalkerSettings &settings);
    void log(string groupname, string name, const WalkerSettings &settings);

    bool print(unsigned skipevery = 0);
    bool dontprint(unsigned skipevery = 0); //removes logging
};

extern shared_ptr<Logger> stdlogger;

class LoadedBinaryLog {
    int len;
public:
    LoadedBinaryLog(string filename);
    float *datastart;
    const int &segmentlength; // how many floats in one segment (all the data saved at one call to print())
    std::vector<string> &log_names;
    std::vector<size_t> offsets;
};


#endif
