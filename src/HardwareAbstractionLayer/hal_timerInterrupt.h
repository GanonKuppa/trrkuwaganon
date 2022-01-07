#pragma once

#include <stdint.h>

namespace hal {
    void initTimerInterrupt0();
    void setPriorityTimerInterrupt0(uint8_t priori);
    void startTimerInterrupt0();
    void stopTimerInterrupt0();

    uint32_t endTimeuCountTimerInterrupt0();
    uint32_t getTimeuCountTimerInterrupt0();

    void setSlot0Time(float usec);
    void setSlot1Time(float usec);
    void setSlot2Time(float usec);
    void setSlot3Time(float usec);

    float getSlot0Time();
    float getSlot1Time();
    float getSlot2Time();
    float getSlot3Time();


    void initTimerInterrupt1();
    void setPriorityTimerInterrupt1(uint8_t priori);
    void restartTimerInterrupt1();
    void startTimerInterrupt1();
    void stopTimerInterrupt1();
    void setInterruptPeriodTimerInterrupt1(uint16_t delay_us);

    uint32_t endTimeuCountTimerInterrupt1();
    uint32_t getTimeuCountTimerInterrupt1();

    void enableMultipleinterrupt();
}