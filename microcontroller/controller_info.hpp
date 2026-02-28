#ifndef SIMPLEWALKER_PICO_CONTROLLER_INFO_HPP
#define SIMPLEWALKER_PICO_CONTROLLER_INFO_HPP

#include "../communication/communication.hpp"
#include "../communication/messages.h"
#include "controller_info.hpp"

// thank you Silverlock on rpi forums for free heap size

// create outbox and initialize temp sensor
MessageOutbox<ControllerInfoMsg> create_controller_info_outbox(Communicator &comm);

void set_send_controller_info(MessageOutbox<ControllerInfoMsg> &controllerInfoOutbox);


#endif //SIMPLEWALKER_PICO_CONTROLLER_INFO_HPP
