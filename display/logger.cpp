#include "logger.hpp"


std::chrono::time_point<std::chrono::steady_clock> timerstart;

shared_ptr<Logger> stdlogger = make_shared<Logger>();

Logger::Logger(string filename, bool log_newline)
    : disable(false), headerSent(false), newline(log_newline), savetofile(false),
      filename(filename), header(""), outstr(""),  lognum(0), skipcntr(0), fieldEndIndexes() {
    if (!filename.empty()) {
        outFile.open(filename, std::ios::trunc);
        if (outFile.is_open()) {
            savetofile = true;
        }
        outFile.close();
    }
}

void Logger::make_field(string name) {
    header += name + "|, ";
    fieldEndIndexes.push_back(std::max(header.size(), outstr.str().size()));
    while (header.size() < fieldEndIndexes.back()) {
        header += " ";
    }
}

void Logger::finish_log_field() {
    while (outstr.str().size() < fieldEndIndexes.at(lognum)) {
        outstr << " ";
    }
    lognum++;
}

void Logger::log(string name, float x) {
    if (disable) return;
    //outstr += std::format("{%9f} |, ", x);
    outstr << std::setw(W) << x << " |, ";
    if (!headerSent) {
        make_field(name);
    }
    finish_log_field();
}

void Logger::log(string name, const float x[], int length) {
    if (disable) return;
    outstr << "( ";
    for (long i = 0; i < length; ++i) {
        outstr << std::setw(W) << x[i] << " ";
    }
    outstr << "), ";
    if (!headerSent) {
        std::stringstream namestr{};
        namestr << name << " [" << length << "]";
        make_field(namestr.str());
    }
    finish_log_field();
}

void Logger::log(string name, const Eigen::Ref<const Eigen::MatrixXf> &x) {
    if (disable) return;
    outstr << "[ ";
    long numcols {x.cols()}, numrows {x.rows()};
    for (long i = 0; i < numrows; ++i) {
        if (numcols > 1 && numrows > 1) outstr << "[";
        for (long j = 0; j < numcols; ++j) {
            outstr << std::setw(W) << x(i, j) << " ";
        }
        if (numcols > 1 && numrows > 1) outstr << "]";
    }
    outstr << "], ";
    if (!headerSent) {
        std::stringstream namestr{};
        namestr << name << " (" << x.rows() << "x" << x.cols() << ")";
        make_field(namestr.str());
    }
    finish_log_field();
}

void Logger::log(string name, const SensorData &x) {
    log(name + "utime", x.timestamp_us);
    log(name + "accel", x.accel, 3);
    log(name + "gyro", x.gyro, 3);
}

void Logger::log(const Logtimes &logtimes) {
    log("predict", logtimes.predict);
    log("correct", logtimes.correct);
    log("comm_rec", logtimes.commreceive);
    log("comm_send", logtimes.commsend);
    log("sensorboss", logtimes.sensorboss);
    log("log", logtimes.log);
    log("sleep", logtimes.sleep);
}

void Logger::log(string name, const WalkerSettings &settings) {
    log(name, settings.f(name.c_str()));
}

void Logger::log(string groupname, string name, const WalkerSettings &settings) {
    log(name, settings.f(groupname.c_str(), name.c_str()));
}

bool Logger::print(unsigned skipevery) {
    bool retval = false;
    if (skipcntr == 0 && !disable) {
        print_line();
        retval = true;
    }
    if (++skipcntr > skipevery) {
        skipcntr = 0;
        disable = false;
    } else {
        disable = true;
    }
    lognum = 0;
    return retval;
}

void Logger::write_file_line_text(string text) {
    outFile.open(filename, std::ios::app);
    if (outFile.is_open()) {
        outFile << text << std::endl;
    }
    outFile.close();
}

void Logger::print_line(void) {
    if (!headerSent) {
        if (savetofile) {
            write_file_line_text(header);
        } else {
            std::cout << header << std::endl;
        }
        headerSent = true;
    }
    if (savetofile) {
        write_file_line_text(outstr.str());
    } else {
        std::cout << outstr.str() << "\r";
        if (newline) {std::cout << "\n";}
        else {std::cout << std::flush;}
    }
    outstr = std::stringstream("");
}

bool Logger::dontprint(unsigned) {
    disable = true;
    return false;
}


// <<<<<<<<<<<<<<<<<<<<<<<<<<< SETTINGS              <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

WalkerSettings::WalkerSettings(string filename) {
    std::ifstream inFile(filename);
    if (inFile.is_open()) {
        text = std::vector<char>((std::istreambuf_iterator<char>(inFile)), std::istreambuf_iterator<char>());
        inFile.close();
        text.push_back('\0');
        doc.parse<rapidxml::parse_trim_whitespace>(&text[0]);
        settings_node = doc.first_node("settings");
        if (!settings_node) std::cerr<<"Couldn't find settings node in "<<filename<<endl;
    }
    else {
        std::cerr<<"Couldn't find "<<filename<<endl;
    }
}

const char *WalkerSettings::cstr(const char * const groupname, const char * const fieldname) const {
    if (!settings_node) return nullptr;
    rapidxml::xml_node<> *node = settings_node->first_node(groupname);
    if (node) {
        node = node->first_node(fieldname);
        if (node) return node->value();
    }
    std::cout<<"Couldn't load "<< groupname << "::" << fieldname <<std::endl;
    return nullptr;
}

const char *WalkerSettings::cstr(const char * const fieldname) const {
    if (!settings_node) return nullptr;
    rapidxml::xml_node<> *node = settings_node->first_node(fieldname);
    if (node) return node->value();
    std::cout<<"Couldn't load "<<fieldname<<std::endl;
    return nullptr;
}

float WalkerSettings::f(const char * const fieldname) const {
    if (!settings_node) return 0.0;
    return std::atof( cstr(fieldname));
}
float WalkerSettings::f(const char * const groupname, const char * const fieldname) const {
    if (!settings_node) return 0.0;
    return std::atof( cstr(groupname, fieldname));
}

int WalkerSettings::i(const char * const fieldname) const {
    if (!settings_node) return 0;
    return std::atoi( cstr(fieldname));
}
int WalkerSettings::i(const char * const groupname, const char * const fieldname) const {
    if (!settings_node) return 0;
    return std::atoi( cstr(groupname, fieldname));
}

bool WalkerSettings::b(const char * const fieldname) const {
    const char * text = cstr(fieldname);
    return (text && std::strlen(text) > 0 && string("false").compare(text) != 0 && string("0").compare(text) != 0);
}
bool WalkerSettings::b(const char * const groupname, const char * const fieldname) const {
    const char * text = cstr(groupname, fieldname);
    return (text && std::strlen(text) > 0 && string("false").compare(text) != 0 && string("0").compare(text) != 0);
}

std::vector<float> strtovf(const char * text) {
    std::vector<float> out;
    if (!text) {return out;}
    char *after;
    while (std::strlen(text) > 0) {
        out.push_back(std::strtof(text, &after));
        text = after;
    }
    return out;
}

std::vector<float> WalkerSettings::vf(const char * const fieldname) const {
    return strtovf(cstr(fieldname));
}
std::vector<float> WalkerSettings::vf(const char * const groupname, const char * const fieldname) const {
    return strtovf(cstr(groupname, fieldname));
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

