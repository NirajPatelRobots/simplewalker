#include "logger.hpp"

#include <iomanip>

const int W = 9;
std::chrono::time_point<std::chrono::steady_clock> timerstart;

Logger::Logger(std::string filename, bool log_newline)
    : disable(false), headerSent(false), newline(log_newline), savetofile(false),
    filename(filename), header(""), outstr(""),  lognum(0), skipcntr(0) {
    if (filename.size() > 0) {
        outFile.open(filename);
        if (outFile.is_open()) {
            savetofile = true;
        }
    }
}

Logger::~Logger() {
    outFile.close();
}

Logger::Logger(bool log_newline) 
    : Logger("", log_newline) {}

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
    log("sleep", logtimes.sleep);
}

void Logger::log(std::string name, const WalkerSettings &settings) {
    log(name, settings.f(name.c_str()));
}

void Logger::log(std::string groupname, std::string name, const WalkerSettings &settings) {
    log(name, settings.f(groupname.c_str(), name.c_str()));
}

bool Logger::print(unsigned skipevery) {
    bool retval = false;
    if (skipcntr == 0 && !disable) {
        if (!headerSent) {
            if (savetofile) {
                outFile << header << std::endl;
            } else {
                std::cout << header << std::endl;
            }
            headerSent = true;
        }
        if (savetofile) {
            outFile << outstr.str() << std::endl;
        } else {
            std::cout << outstr.str() << "\r";
            if (newline) {std::cout << "\n";}
            else {std::cout << std::flush;}
        }
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


// <<<<<<<<<<<<<<<<<<<<<<<<<<< SETTINGS              <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

WalkerSettings::WalkerSettings(std::string filename) {
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        text = std::vector<char>((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
        inFile.close();
        text.push_back('\0');
        doc.parse<rapidxml::parse_trim_whitespace>(&text[0]);
    }
    else {
        std::cerr<<"Couldn't find "<<filename<<std::endl;
    }
}

float WalkerSettings::f(const char * const fieldname) const {
    rapidxml::xml_node<> *node = doc.first_node(fieldname);
    if (node) {
        return std::atof(node->value());
    }
    std::cout<<"Couldn't load "<<fieldname<<std::endl;
    return 0;
}

float WalkerSettings::f(const char * const groupname, const char * const fieldname) const {
    rapidxml::xml_node<> *node = doc.first_node(groupname);
    if (node) {
        node = node->first_node(fieldname);
        if (node) return std::atof(node->value());
    }
    std::cout<<"Couldn't load "<< groupname << "::" << fieldname <<std::endl;
    return 0;
}

bool WalkerSettings::b(const char * const fieldname) const {
    rapidxml::xml_node<> *node = doc.first_node(fieldname);
    if (node) {
        return (node->value_size() > 0 && std::string("false").compare(node->value()) != 0);
    }
    std::cout<<"Couldn't load "<<fieldname<<std::endl;
    return false;
}

bool WalkerSettings::b(const char * const groupname, const char * const fieldname) const {
    rapidxml::xml_node<> *node = doc.first_node(groupname);
    if (node) {
        node = node->first_node(fieldname);
        if (node) {
            return (node->value_size() > 0 && std::string("false").compare(node->value()) != 0
                    && std::string("0").compare(node->value()) != 0);
        }
        std::cout<<"Couldn't load "<<fieldname<<std::endl;
    }
    std::cout<<"Couldn't load "<< groupname << "::" << fieldname << std::endl;
    return false;
}


// <<<<<<<<<<<<<<<<<<<<<<<<<<< LOG TIMING              <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
namespace chrono = std::chrono;
void start_logtiming(chrono::time_point<chrono::steady_clock> time) {
    timerstart = time;
}

void set_logtime(float &logtime) {
    chrono::time_point<chrono::steady_clock> now = chrono::steady_clock::now();
    logtime = chrono::duration<float, std::milli>(now - timerstart).count();
    timerstart = now;
}

