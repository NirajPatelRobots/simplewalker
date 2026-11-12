/* TODO:
    Have motor model json define motor model, and logic converts it to commands
        json -> MotorModel object -> (logic) -> commands to create that motor model
        then MotorModel -> commands can be reused by Read Model API
    Reset model command
*/
#include <fstream>
#include <nlohmann/json.hpp>
#include "comm_serial.hpp"
#include "communication_timer.hpp"
#include "motor_model_file_IO.hpp"

using std::string, MotorModelAPI::Request, MotorModelAPI::Response;


const chrono::microseconds TIMEOUT_US{2'000'000};
const unsigned             NUM_RETRIES{4};

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
    
    void send_and_receive(const Request &request) {
        APIOutbox.message = request;
        for (unsigned tries = 0; tries < NUM_RETRIES; tries++) {
            controller_comm->clear_buffer();
            APIOutbox.send();
            listen_timeout(*controller_comm, TIMEOUT_US);
            if (APIInbox.num_available() > 0) 
                break;
            printf("Try again...%u\n", tries + 1);
        }
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

    std::vector<Request> requests;
    parse_json_file(filename, requests);

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
