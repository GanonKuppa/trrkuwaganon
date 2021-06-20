#pragma once

#include <stdint.h>

namespace periferal_driver {
    void initMTU1();
    void initMTU2();
    uint16_t getCountMTU1();
    uint16_t getCountMTU2();
}