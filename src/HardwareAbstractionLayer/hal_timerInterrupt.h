#pragma once

#include <stdint.h>

namespace hal {
    void initTimerInterrupt0();
    void setPriorityTimerInterrupt0(uint8_t priori);
    void startTimerInterrupt0();
    void stopTimerInterrupt0();

    uint32_t endTimeuCountTimerInterrupt0();
    uint32_t getTimeuCountTimerInterrupt0();

    void setSlot0Time(uint32_t usec);
    void setSlot1Time(uint32_t usec);
    void setSlot2Time(uint32_t usec);
    void setSlot3Time(uint32_t usec);

    uint32_t getSlot0Time();
    uint32_t getSlot1Time();
    uint32_t getSlot2Time();
    uint32_t getSlot3Time();


    void initTimerInterrupt1();
    void setPriorityTimerInterrupt1(uint8_t priori);
    void startTimerInterrupt1();
    void stopTimerInterrupt1();

    uint32_t endTimeuCountTimerInterrupt1();
    uint32_t getTimeuCountTimerInterrupt1();
}