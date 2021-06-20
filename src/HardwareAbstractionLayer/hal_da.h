#pragma once

#include <stdint.h>

namespace hal {
    void initDA();
    void setDA(float da);
    float getDA();
    constexpr uint16_t DA_RESOLUTION = 4096;
}