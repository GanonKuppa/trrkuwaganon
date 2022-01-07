#include <stdint.h>
#include "hal_ad.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_ad.h>
#endif

namespace hal {
    void initAD() {
#ifndef SILS
        peripheral_driver::initAD();
#endif
    }

    uint16_t startAD0() {
#ifndef SILS
        return peripheral_driver::startAD_AN109();
#else
        return 0;
#endif

    }

    uint16_t startAD1() {
#ifndef SILS
        return peripheral_driver::startAD_AN101();
#else
        return 0;

#endif

    }

    uint16_t startAD2() {
#ifndef SILS
        return peripheral_driver::startAD_AN111();
#else
        return 0;

#endif

    }

    uint16_t startAD3() {
#ifndef SILS
        return peripheral_driver::startAD_AN110();
#else
        return 0;
#endif

    }

    uint16_t startAD4() {
#ifndef SILS
        return peripheral_driver::startAD_AN108();
#else
        return 0;
#endif

    }

    //uint16_t startAD5();
    //uint16_t startAD6();
    //uint16_t startAD7();

    uint16_t getAD0() {
#ifndef SILS
        return peripheral_driver::getAD_AN109();
#else
        return 0;
#endif

    }

    uint16_t getAD1() {
#ifndef SILS
        return peripheral_driver::getAD_AN101();
#else
        return 0;
#endif

    }

    uint16_t getAD2() {
#ifndef SILS
        return peripheral_driver::getAD_AN111();
#else
        return 0;
#endif

    }

    uint16_t getAD3() {
#ifndef SILS
        return peripheral_driver::getAD_AN110();
#else
        return 0;
#endif

    }

    uint16_t getAD4() {
#ifndef SILS
        return peripheral_driver::getAD_AN108();
#else
        return 0;
#endif

    }

    //uint16_t getAD5();
    //uint16_t getAD6();
    //uint16_t getAD7();



}