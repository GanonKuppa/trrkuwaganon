#pragma once 
#include "communication.h"

#define PRINTF_ASYNC(_text, ...) \
    do { \
        module::Communication::getInstance().printfAsync(_text,  ##__VA_ARGS__); \
    } while(0);


#define PRINTF_SYNC(_text, ...) \
    do { \
        module::Communication::getInstance().printfSync(_text,  ##__VA_ARGS__); \
    } while(0);
