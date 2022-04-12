/*pico 2 way communication with the rasberry pi zero (main computer)
(LSB first)
*/

#include "pico/stdlib.h"

bool read_struct(char *buff, size_t length, uint16_t ID, int timeout);

void send_struct(char *buff, size_t length);

void pico_comm_init(void); // wraps stdio_init_all()
