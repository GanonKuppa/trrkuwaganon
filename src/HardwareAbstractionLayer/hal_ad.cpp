#include <stdint.h>
#include "hal_ad.h"

#ifndef SILS
#include "pd_ad.h"
#endif

namespace hal {
    void initAD() {
#ifndef SILS
        periferal_driver::initAD();
#endif
    }

    uint16_t startAD0() {
#ifndef SILS
        return periferal_driver::startAD_AN001();
#else
        return 0;
#endif

    }

    uint16_t startAD1() {
#ifndef SILS
        return periferal_driver::startAD_AN101();
#else
        return 0;

#endif

    }

    uint16_t startAD2() {
#ifndef SILS
        return periferal_driver::startAD_AN108();
#else
        return 0;

#endif

    }

    uint16_t startAD3() {
#ifndef SILS
        return periferal_driver::startAD_AN005();
#else
        return 0;
#endif

    }

    uint16_t startAD4() {
#ifndef SILS
        return periferal_driver::startAD_AN000();
#else
        return 0;
#endif

    }

    //uint16_t startAD5();
    //uint16_t startAD6();
    //uint16_t startAD7();

    uint16_t getAD0() {
#ifndef SILS
        return periferal_driver::getAD_AN001();
#else
        return 0;
#endif

    }

    uint16_t getAD1() {
#ifndef SILS
        return periferal_driver::getAD_AN101();
#else
        return 0;
#endif

    }

    uint16_t getAD2() {
#ifndef SILS
        return periferal_driver::getAD_AN108();
#else
        return 0;
#endif

    }

    uint16_t getAD3() {
#ifndef SILS
        return periferal_driver::getAD_AN005();
#else
        return 0;
#endif

    }

    uint16_t getAD4() {
#ifndef SILS
        return periferal_driver::getAD_AN000();
#else
        return 0;
#endif

    }

    //uint16_t getAD5();
    //uint16_t getAD6();
    //uint16_t getAD7();



}