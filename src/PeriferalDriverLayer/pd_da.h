#pragma once

#include <stdint.h>

namespace periferal_driver {
    void initDA();
    void setDA(uint16_t da);
    uint16_t getDA();
}
