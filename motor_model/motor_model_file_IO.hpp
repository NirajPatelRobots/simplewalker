#include "motor_model_API.hpp"
#include <stdexcept>


class MotorModelAPIException : public std::runtime_error {
public:
    MotorModelAPIException(const std::string& msg) : std::runtime_error(msg) {}
};


void parse_json_file(const std::string &filename, std::vector<MotorModelAPI::Request> &requests_out);
