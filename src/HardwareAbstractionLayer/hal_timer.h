#pragma once

#include <stdint.h>

namespace hal {

    void initTimer();
    void waitClockCount(uint32_t cCount);
    void waitusec(uint32_t usec);
    void waitmsec(uint32_t msec);
    void startTimeuCount();
    uint32_t getTimeuCount();
    uint32_t endTimeuCount();

    void waitClockCount_sub(uint32_t cCount);
    void waitusec_sub(uint32_t usec);
    void waitmsec_sub(uint32_t msec);
    void startTimeuCount_sub();
    uint32_t getTimeuCount_sub();
    uint32_t endTimeuCount_sub();

    uint32_t getElapsedMsec();
    uint32_t getElapsedSec();


}
