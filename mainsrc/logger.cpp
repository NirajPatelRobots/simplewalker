#include "logger.hpp"

#include <iomanip>
#include <fstream>

const int W = 9;
std::chrono::time_point<std::chrono::steady_clock> timerstart;

Logger::Logger(std::string filename, int logoptions)
    : disable(false), headerSent(false), filename(filename),
     header(""), outstr(""), options(logoptions), lognum(0), skipcntr(0) {
    if (filename.size() > 0) {
        //open file
        //ostream = ;
    }
}

Logger::Logger(int logoptions) 
    : Logger("", logoptions) {}

void Logger::log(std::string name, float x) {
    if (disable) return;
    //outstr += std::format("{%9f} |, ", x);
    outstr << std::setw(W) << x << " |, ";
    if (!headerSent) {
        header += name + ", ";
        endindexes.push_back(std::max(header.size(), outstr.str().size()));
        while (header.size() < endindexes.back()) {
            header += " ";
        }
    }
    while (outstr.str().size() < endindexes.at(lognum)) {
        outstr << " ";
    }
    lognum++;
}

void Logger::log(std::string name, const Vector3f &x) {
    if (disable) return;
    //outstr += std::format("[{%9f} {%9f} {%9f}] |, ", x(0), x(1), x(2));
    outstr << std::setw(W) << x(0) << " " << std::setw(W) << x(1) << " " << std::setw(W) << x(2) << " |, ";
    if (!headerSent) {
        header += name + ", ";
        endindexes.push_back(std::max(header.size(), outstr.str().size()));
        while (header.size() < endindexes.back()) {
            header += " ";
        }
    }
    while (outstr.str().size() < endindexes.at(lognum)) {
        outstr << " ";
    }
    lognum++;
}

void Logger::log(std::string name, const RobotState &x) {
    log(name + " pos", x.pos());
    log(name + " euler", x.euler());
    log(name + " vel", x.vel());
    log(name + " angvel", x.angvel());
}

void Logger::log(std::string name, const SensorBoss &x) {
    log(name + " accel", x.accel());
    log(name + " gyro", x.gyro());
}

void Logger::log(const Logtimes &logtimes) {
    log("predict", logtimes.predict);
    log("correct", logtimes.correct);
    log("commreceive", logtimes.commreceive);
    log("log", logtimes.log);
}

bool Logger::print(unsigned skipevery) {
    bool retval = false;
    if (skipcntr == 0 && !disable) {
        if (!headerSent) {
            std::cout << header << std::endl;
            headerSent = true;
        }
        std::cout << outstr.str() << "\r";
        if (options & LOGNEWLINE) {std::cout << "\n";}
        else {std::cout << std::flush;}
        outstr = std::stringstream("");
        retval = true;
    }
    if (skipcntr++ > skipevery) {
        skipcntr = 0;
        disable = false;
    } else {
        disable = true;
    }
    lognum = 0;
    return retval;
}

bool Logger::dontprint(unsigned) {
    disable = true;
    return false;
}

    
namespace chrono = std::chrono;
void start_logtiming(chrono::time_point<chrono::steady_clock> time) {
    timerstart = time;
}

void set_logtime(float &logtime) {
    chrono::time_point<chrono::steady_clock> now = chrono::steady_clock::now();
    logtime = chrono::duration<float, std::milli>(now - timerstart).count();
    timerstart = now;
}

