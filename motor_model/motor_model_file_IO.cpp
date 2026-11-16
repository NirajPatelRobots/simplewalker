#include "motor_model_file_IO.hpp"
#include <fstream>
#include <nlohmann/json.hpp>


using nlohmann::json, std::string, MotorModelAPI::Request, MotorModelAPI::Response;


template <typename T>
T get_or_throw(const json &entry, const string &entry_key) {
    if (!entry.contains(entry_key)) {
        throw MotorModelAPIException("Entry is missing " + entry_key + ": " + entry.dump());
    }
    return entry[entry_key].get<T>();
}


void parse_json(nlohmann::json &data, std::vector<Request> &requests) {
    for (auto &entry : data["requests"]) {
        MotorModelAPI::GROUP group = MotorModelAPI::GROUP::STATE;
        if (entry.contains("group")) {
            string group_str = entry["group"].get<string>();
            if (group_str == "input") {
                group = MotorModelAPI::GROUP::INPUT;
            } else if (group_str != "state") {
                throw MotorModelAPIException("Group must be 'input' or 'state', got " + group_str);
            }
        }
        string cmd = get_or_throw<string>(entry, "cmd");
        requests.emplace_back();
        if (cmd == "set_function") {
            string fcn_name = get_or_throw<string>(entry, "fcn_name");
            int term_idx = get_or_throw<int>(entry, "term_idx");
            int fcn_idx = get_or_throw<int>(entry, "fcn_idx");
            if (!MotorModelAPI::msg_set_function(group, term_idx, fcn_idx, fcn_name.c_str(), requests.back())) {
                throw MotorModelAPIException("Error creating set_function message. Check fcn_name.");
            }
        } else if (cmd == "set_parameter") {
            int term_idx = get_or_throw<int>(entry, "term_idx");
            int fcn_idx = get_or_throw<int>(entry, "fcn_idx");
            int param_idx = get_or_throw<int>(entry, "param_idx");
            float value = get_or_throw<float>(entry, "value");
            if (!MotorModelAPI::msg_set_parameter(group, term_idx, fcn_idx, param_idx, value, requests.back())) {
                throw MotorModelAPIException("Error creating set_parameter message");
            }
        } else if (cmd == "set_weight") {
            int term_idx = get_or_throw<int>(entry, "term_idx");
            float weight = get_or_throw<float>(entry, "weight");
            if (!MotorModelAPI::msg_set_weight(group, term_idx, weight, requests.back())) {
                throw MotorModelAPIException("Error creating set_weight message");
            }
        } else {
            throw MotorModelAPIException("Invalid cmd: " + cmd);
        }
        printf("Created message for cmd %s\n", cmd.c_str());
    }
}


void parse_json_file(const std::string &filename, std::vector<MotorModelAPI::Request> &requests) {
    std::ifstream file(filename);
    if (file.fail()) {
        throw std::runtime_error("Error, could not open " + filename);
    }

    json data;
    try {
        data = json::parse(file);
    } catch (const json::exception &e) {
        throw std::runtime_error(std::string("Error parsing json: \n\t") + e.what());
    }
    file.close();
    try {
        parse_json(data, requests);
    } catch (const MotorModelAPIException &e) {
        throw std::runtime_error(std::string("Error creating message: \n\t") + e.what());
    }
}
