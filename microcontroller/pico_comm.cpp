#include "pico_comm.hpp"
#include <stdio.h>

bool read_struct(char *buff, size_t length, uint16_t ID, int timeout) {
    /* buff is what to write to, length is size of buff, ID is message ID to expect, timeout is per byte in us.
    returns bool for success */
    static int bytesReceived = 0;
    while (1)
    {
        int inChar = getchar_timeout_us(timeout);
        if (inChar == PICO_ERROR_TIMEOUT) {
            return false;
        }
        buff[bytesReceived++] = (inChar & 0xFF); // this feels like real C coding
        if (bytesReceived == 2 && *(uint16_t *)buff != ID) {
            bytesReceived = 1;
            return false; //wrong ID
        }
        if (bytesReceived == length) {
            bytesReceived = 0;
            return true;
        }
    }
}

void send_struct(char *buff, size_t length) {
    for (int i = 0; i < length; i++) {
        putchar_raw(buff[i]);
    }
}

void pico_comm_init(void) {
    stdio_init_all();
}


