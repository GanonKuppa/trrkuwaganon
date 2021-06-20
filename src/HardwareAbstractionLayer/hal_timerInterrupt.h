#pragma once

#include <stdint.h>

namespace hal {
    void initTimerInterrupt0();
    void setPriorityTimerInterrupt0(uint8_t priori);
    void startTimerInterrupt0();
    void stopTimerInterrupt0();

    uint32_t endTimeuCountTimerInterrupt0();
    uint32_t getTimeuCountTimerInterrupt0();

    void initTimerInterrupt1();
    void setPriorityTimerInterrupt1(uint8_t priori);
    void startTimerInterrupt1();
    void stopTimerInterrupt1();

    uint32_t endTimeuCountTimerInterrupt1();
    uint32_t getTimeuCountTimerInterrupt1();
}