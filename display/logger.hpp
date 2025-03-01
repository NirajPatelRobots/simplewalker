/*
Logging simplewalker data

Can output:
    header to explain other data
    relevant data of many objects
    human readable or raw data
    Can output to stdout or file
    
TODO:
    BUG: not always the right width
    Fix error if print() not called again before more logs
    Fancy macro or something so logging doesn't happen?
    logging to file is pretty slow
    Binary logger?
    multiple lines to text file
    settings fieldname with no groupname finds lowest level fieldname
    add/remove prefix from logger name for surrounding code blocks so we can tell logs come from that block
*/
#ifndef SIMPLEWALKER_LOGGER
#define SIMPLEWALKER_LOGGER
#include <sstream>
#include <vector>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <deque>
#include <iomanip>
#include "walkerUtils.hpp"
#include "rapidxml.hpp"


class WalkerSettings {
    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> *settings_node = nullptr;
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
    std::vector<float> vf(const char * const fieldname) const;
    std::vector<float> vf(const char * const groupname, const char * const fieldname) const;
};


struct Logtimes {
    float predict, correct, commreceive, commsend, sensorboss, log, sleep;
};

void start_logtiming(std::chrono::time_point<std::chrono::steady_clock> time); //starts timing, logtime is relative to this
void set_logtime(float &logtime); //sets this log time and starts counting from 0 again

class Logger {
protected:
    int W = 9;
    bool disable, headerSent, newline, savetofile;
    std::ofstream outFile;
    std::string filename, header;
    std::stringstream outstr;
    unsigned lognum, skipcntr;
    std::vector<unsigned> fieldEndIndexes;
    void make_field(string name);
    void print_line();
    void write_file_line_text(string text);
    void finish_log_field();
public:
    Logger(string filename = "", bool log_newline = false);  //default file is stdout
    void log(string name, float x);
    void log(string name, const float x[], int length);
    void log(string name, const Eigen::Ref<const Eigen::MatrixXf> &x);
    void log(string name, const SensorData &x);
    void log(const SensorData &x) {log("", x);}
    void log(const Logtimes &logtimes);
    void log(string name, const WalkerSettings &settings);
    void log(string groupname, string name, const WalkerSettings &settings);
    template<typename T>
    void log(const string &name, const std::deque<T> &x) {
        if (disable) return;
        outstr << "[ ";
            for (const T &x_i : x) outstr << std::setw(W) << x_i << " ";
        outstr << "], ";
        if (!headerSent)
            make_field(name + " [" + std::to_string(x.size()) + "]");
        finish_log_field();
    }
    template<typename T>
    void log(const string &name, const std::vector<T> &x) {
        if (disable) return;
        outstr << "[ ";
        for (const T &x_i : x) outstr << std::setw(W) << x_i << " ";
        outstr << "], ";
        if (!headerSent)
            make_field(name + " [" + std::to_string(x.size()) + "]");
        finish_log_field();
    }

    bool print(unsigned skipevery = 0);
    bool dontprint(unsigned skipevery = 0); //removes logging
    string get_filename(void) {return filename;}
    inline bool is_saving_to_file(void) {return savetofile;}
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
