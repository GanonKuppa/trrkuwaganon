#pragma once 
#include "hal_uart.h"

#define PRINTF_SYNC(_text, ...) \
    do { \
        hal::printfSync(_text,  ##__VA_ARGS__); \
    } while(0);


#define PRINTF_ASYNC(_text, ...) \
    do { \
        hal::printfAsync(_text,  ##__VA_ARGS__); \
    } while(0);



