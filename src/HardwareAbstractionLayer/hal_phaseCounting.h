#pragma once

#include <stdint.h>

namespace hal {
    void initPhaseCounting0();
    void initPhaseCounting1();
    uint16_t getPhaseCount0();
    uint16_t getPhaseCount1();
}