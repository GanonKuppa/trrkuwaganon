#include "hal_clock.h"

#ifndef SILS
#include "pd_clock.h"
#endif

namespace hal {
    void initClock() {
#ifndef SILS
        periferal_driver::initClock();
#endif
    }
}

