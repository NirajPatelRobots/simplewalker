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


const unsigned TIMEOUT_US{10'000'000};


void print_response(const Response &response) {
    printf("Status %s, cmd %s, group %u, term_idx %u, fcn_idx %u, fcn_field %u\n",
        MotorModelAPI::response_status_name((Response::STATUS)response.status),
        request_command_name(response.command), (uint8_t)response.group,
        response.term_idx, response.fcn_idx, response.fcn_field);
}

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
            controller_comm->clear_buffer();
            controller_comm->clear_all_messages();
        }
    
    void send_and_receive(const Request &request, bool no_send) {
        response.status = Response::UNSET;
        APIOutbox.message = request;
        controller_comm->clear_buffer();
        if (!no_send)   APIOutbox.send();
        int64_t listen_us = listen_timeout(APIInbox, TIMEOUT_US);
        if (APIInbox.get_newest(response) < 0) {
            throw MotorModelAPIException("Did not get a response");
        }
        printf("Got response after %.2f s: %s\n", listen_us * 1e-6,
            MotorModelAPI::response_status_name((Response::STATUS)response.status));
        if (response.status != Response::STATUS::SUCCESS) {
            print_response(response);
            throw MotorModelAPIException("Unsuccessful response from Target");
        }
        if (response.command != request.command || response.group != request.group
                || response.term_idx != request.term_idx || response.fcn_idx != request.fcn_idx
                || response.fcn_field != request.fcn_field) {
            print_response(response);
            throw MotorModelAPIException("Response did not match request");
        }
    }
};


int main(int argc, char *argv[]) {
    string filename = "settings/motor_model.json";
    const string USAGE_HELP = "Usage: motor_model_user_API [--check] [--no-send] [-timeout=10] filename";
    bool dry_run = false, no_send = false;
    float timeout = 10;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dry-run") == 0 || strcmp(argv[i], "--check") == 0) {
            dry_run = true;
        } else if (strcmp(argv[i], "--no-send") == 0) {
            no_send = true;
        } else if (strncmp(argv[i], "--timeout=", 10) == 0) {
            timeout = strtof(argv[i] + 10, nullptr);
            if (timeout <= 0) {
                printf("Got bad timeout: %f\n", timeout);
                return 2;
            }
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
    if (no_send)
        printf("Will listen, but won't send messages\n");

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
            printf("\nSending message %s: group %u, term_idx %u, fcn_idx %u, fcn_field %u\n",
                request_command_name(request.command), (uint8_t)request.group, request.term_idx, request.fcn_idx, request.fcn_field);
            comms.send_and_receive(request, no_send);
        } catch (const MotorModelAPIException &e) {
            printf("Error: %s\n", e.what());
            if (!no_send) return 3;
        }
    }

}
