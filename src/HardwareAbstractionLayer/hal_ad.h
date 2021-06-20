#pragma once

#include <stdint.h>

namespace hal {
    void initAD();

    uint16_t startAD0();
    uint16_t startAD1();
    uint16_t startAD2();
    uint16_t startAD3();
    uint16_t startAD4();
    //uint16_t startAD5();
    //uint16_t startAD6();
    //uint16_t startAD7();

    uint16_t getAD0();
    uint16_t getAD1();
    uint16_t getAD2();
    uint16_t getAD3();
    uint16_t getAD4();
    //uint16_t getAD5();
    //uint16_t getAD6();
    //uint16_t getAD7();

}
