/*
Logging more advanced simplewalker information, child class of Logger
TODO:
    cpp file
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
    void obj_log(string name, const SensorData &x);
    void obj_log(const SensorData &x) {obj_log("", x);}
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

void ConvenientLogger::obj_log(string name, const SensorData &x) {
    log(name + "time", x.timestamp_us * 1e6);
    log(name + "accel", x.accel, 3);
    log(name + "gyro", x.gyro, 3);
}

#endif
