/* TODO:
    Have motor model json define motor model, and logic converts it to commands?
        json -> MotorModel object -> (logic) -> commands to create that motor model?
        then MotorModel -> commands can be reused by Read Model API
    Reset model command
    command names hardcoded in motor_model_API_user.cpp
*/
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include "comm_serial.hpp"
#include "communication_timer.hpp"
#include "messages.h"
#include "motor_model_API.hpp"

using nlohmann::json, std::string, MotorModelAPI::Request, MotorModelAPI::Response;


class MotorModelAPIException : public std::runtime_error {
public:
    MotorModelAPIException(const string& msg) : std::runtime_error(msg) {}
};

const chrono::microseconds timeout_us{2'000'000};

struct motorModelUserAPIComm {
    std::unique_ptr<SerialCommunicator> controller_comm;
    MessageInbox<Response> APIInbox;
    MessageOutbox<Request> APIOutbox;
    Response response;

    motorModelUserAPIComm() :
        controller_comm(new SerialCommunicator(string("Controller_serial"))),
        APIInbox(MotorModelAPIResponseID, *controller_comm),
        APIOutbox(MotorModelAPIRequestID, *controller_comm),
        response() {
            controller_comm->clear_all_messages();
        }
    
    void send_and_receive(Request &request) {
        APIOutbox.message = request;
        APIOutbox.send();
        listen_timeout(*controller_comm, timeout_us);
        if (APIInbox.get_newest(response) < 0) {
            throw MotorModelAPIException("Did not get a response");
        }
        printf("Got response: term_idx %d, status %s\n", response.term_idx,
            MotorModelAPI::response_status_name((Response::STATUS)response.status));
        if (response.status != Response::STATUS::SUCCESS) {
            throw MotorModelAPIException("Unsuccessful response from Target");
        }
    }
};

template <typename T>
T get_or_throw(const json &entry, const string &entry_key) {
    if (!entry.contains(entry_key)) {
        throw MotorModelAPIException("Entry is missing " + entry_key + ": " + entry.dump());
    }
    return entry[entry_key].get<T>();
}

// Read json data, either create a list of requests or throw an exception
void parse_json(json &data, std::vector<Request> &requests) {
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
        if (cmd == "create_term") {
            string fcn_name = get_or_throw<string>(entry, "fcn_name");
            float weight = get_or_throw<float>(entry, "weight");
            if (!MotorModelAPI::msg_create_term(group, fcn_name.c_str(), weight, requests.back())) {
                throw MotorModelAPIException("create_term. Check fcn_name.");
            }
        } else if (cmd == "delete_term") {
            int term_idx = get_or_throw<int>(entry, "term_idx");
            if (!MotorModelAPI::msg_delete_term(group, term_idx, requests.back())) {
                throw MotorModelAPIException("delete_term");
            }
        } else if (cmd == "add_function") {
            string fcn_name = get_or_throw<string>(entry, "fcn_name");
            int term_idx = get_or_throw<int>(entry, "term_idx");
            if (!MotorModelAPI::msg_add_function(group, term_idx, fcn_name.c_str(), requests.back())) {
                throw MotorModelAPIException("add_function. Check fcn_name.");
            }
        } else if (cmd == "set_parameter") {
            int term_idx = get_or_throw<int>(entry, "term_idx");
            int fcn_idx = get_or_throw<int>(entry, "fcn_idx");
            int param_idx = get_or_throw<int>(entry, "param_idx");
            float value = get_or_throw<float>(entry, "value");
            if (!MotorModelAPI::msg_set_parameter(group, term_idx, fcn_idx, param_idx, value, requests.back())) {
                throw MotorModelAPIException("set_parameter");
            }
        } else if (cmd == "set_weight") {
            int term_idx = get_or_throw<int>(entry, "term_idx");
            float weight = get_or_throw<float>(entry, "weight");
            if (!MotorModelAPI::msg_set_weight(group, term_idx, weight, requests.back())) {
                throw MotorModelAPIException("set_weight");
            }
        } else {
            throw MotorModelAPIException("Invalid cmd: " + cmd);
        }
        printf("Created message for cmd %s\n", cmd.c_str());
    }
}


int main(int argc, char *argv[]) {
    string filename = "settings/motor_model.json";
    const string USAGE_HELP = "Usage: motor_model_user_API [--check] filename";
    bool dry_run = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dry-run") == 0 || strcmp(argv[i], "--check") == 0) {
            dry_run = true;
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            printf("%s\n", USAGE_HELP.c_str());
            return 0;
        } else if (argv[i][0] == '-') {
            printf("Unrecognized argument: %s\n%s\n", argv[i], USAGE_HELP.c_str());
            return 1;
        } else {
            filename = argv[i];
        }
    }
    printf("Loading motor model from %s\n", filename.c_str());
    if (dry_run)
        printf("Check only: Will check file but won't start communication\n");
    
    std::ifstream file(filename);
    if (!file.is_open()) {
        fprintf(stderr, "Error, could not open %s\n", filename.c_str());
        return 1;
    }

    json data;
    try {
        data = json::parse(file);
    } catch (const json::exception &e) {
        printf("Error parsing json:\n\t%s\n", e.what());
        return 2;
    }
    std::vector<Request> requests;
    try {
        parse_json(data, requests);
    } catch (const MotorModelAPIException &e) {
        printf("Error creating message:\n\t%s\n", e.what());
        return 3;
    }
    if (dry_run) {
        printf("Parsed input, check completed, exiting\n");
        return 0;
    }
    printf("Parsed input, connecting to microcontroller...\n");

    motorModelUserAPIComm comms;

    for (auto & request : requests) {
        try {
            printf("Sending message %s\n", request_command_name(request.command));
            comms.send_and_receive(request);
        } catch (const MotorModelAPIException &e) {
            printf("Error: %s\n", e.what());
            return 3;
        }
    }

}
