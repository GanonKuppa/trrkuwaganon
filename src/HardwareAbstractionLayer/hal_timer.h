#pragma once

#include <stdint.h>

namespace hal {

    void initTimer();
    void waitClockCount(uint32_t cCount);
    void waitnsec(uint32_t nsec);
    void waitusec(uint32_t usec);
    void waitmsec(uint32_t msec);

    uint64_t getElapsedClockCount();
    uint64_t getElapsedNsec();
    uint64_t getElapsedUsec();
    uint32_t getElapsedMsec();
    uint32_t getElapsedSec();
    float calcElapsedUsec(uint64_t clock_count);

    void hrtStartTimer();
    void hrtStopTimer();
    float hrtGetElapsedUsec(float usec=0.0f);
}
