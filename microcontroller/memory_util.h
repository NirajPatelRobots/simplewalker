#ifndef SIMPLEWALKER_PICO_MEMORY_UTIL_H
#define SIMPLEWALKER_PICO_MEMORY_UTIL_H

#include <malloc.h>

// thank you Silverlock for https://forums.raspberrypi.com/viewtopic.php?t=347638#p2082565

inline uint32_t getTotalHeap() {
    extern char __StackLimit, __bss_end__;
    return &__StackLimit  - &__bss_end__;
}

inline uint32_t getFreeHeapSize() {
    struct mallinfo m = mallinfo();
    return getTotalHeap() - m.uordblks;
}

#endif //SIMPLEWALKER_PICO_MEMORY_UTIL_H
