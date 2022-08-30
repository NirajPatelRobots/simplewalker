/*
Logging more advanced simplewalker information, child class of Logger

Can output:
    header to explain other data
    relevant data of many objects
    human readable or raw data
    Can output to stdout or file
    
*/
#ifndef SIMPLEWALKER_CONVENIENT_LOGGER
#define SIMPLEWALKER_CONVENIENT_LOGGER
#include "logger.hpp"
#include "robot_state.hpp"
#include "sensorBoss.hpp"

class ConvenientLogger : public Logger {
public:
    explicit ConvenientLogger(const string name) : Logger(name, true) {}
    void obj_log(std::string name, const RobotState &x);
    void obj_log(const RobotState &x) {obj_log("", x);}
    void obj_log(std::string name, const SensorBoss &x);
    void obj_log(const SensorBoss &x) {obj_log("", x);}
};

void ConvenientLogger::obj_log(std::string name, const RobotState &x) {
    Logger::log(string(name + " pos"), x.pos());
    Logger::log(name + " axis", x.axis());
    Logger::log(name + " vel", x.vel());
    Logger::log(name + " angvel", x.angvel());
}

void ConvenientLogger::obj_log(std::string name, const SensorBoss &x) {
    Logger::log(name + " accel", x.accel());
    Logger::log(name + " gyro", x.gyro());
    Logger::log(name + " accel_pred", x.accel_pred());
    Logger::log(name + " gyro_pred", x.gyro_pred());
}

#endif
