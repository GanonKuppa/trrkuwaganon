#pragma once

#include <stdint.h>

namespace peripheral_driver {

    void initCMTW0();
    void initCMTW1();
    void initTPU0();

    void startCMTW();

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

    // 以下の関数はスレッドセーフではないので同一タイマ割り込み関数内でのみコールすること
    void hrtStartTimer();
    void hrtStopTimer();
    float hrtGetElapsedUsec(float usec=0.0f);

}
