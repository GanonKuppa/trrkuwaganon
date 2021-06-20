#include <stdint.h>
#include "hal_phaseCounting.h"

#ifndef SILS
#include "pd_phaseCounting.h"
#endif

namespace hal {
    void initPhaseCounting0() {
#ifndef SILS
        periferal_driver::initMTU1();
#endif
    }

    void initPhaseCounting1() {
#ifndef SILS
        periferal_driver::initMTU2();
#endif
    }

    uint16_t getPhaseCount0() {
#ifndef SILS
        return periferal_driver::getCountMTU1();
#else
        return 0;
#endif
    }

    uint16_t getPhaseCount1() {
#ifndef SILS
        return periferal_driver::getCountMTU2();
#else
        return 0;
#endif
    }
}