#pragma once

#include <stdint.h>

namespace periferal_driver {

    constexpr uint32_t ICLK = 96000000;
    constexpr uint32_t BCLK = 96000000;
    constexpr uint32_t PCLKA = 96000000;
    constexpr uint32_t PCLKB = 48000000;
    constexpr uint32_t PCLKC = 48000000;
    constexpr uint32_t PCLKD = 48000000;

    void initClock();
}
