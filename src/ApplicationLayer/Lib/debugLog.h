#pragma once 
#include "hal_uart.h"
#include "hal_timer.h"

#define PRINTF_SYNC(_text, ...) \
    do { \
        hal::printfSync(_text,  ##__VA_ARGS__); \
    } while(0);


#define PRINTF_ASYNC(_text, ...) \
    do { \
        hal::printfAsync(_text,  ##__VA_ARGS__); \
    } while(0);


#define PRINTF_PICKLE(_text, ...) \
    do { \
        uint32_t elapsed_msec = hal::getElapsedMsec();\
        hal::printfPickle("%010d| ",elapsed_msec);\
        hal::printfPickle(_text,  ##__VA_ARGS__); \
    } while(0);
