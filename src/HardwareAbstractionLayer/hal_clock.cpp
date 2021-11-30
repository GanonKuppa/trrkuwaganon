#include "hal_clock.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_clock.h>
#endif

namespace hal {
    void initClock() {
#ifndef SILS
        peripheral_driver::initClock();
#endif
    }
}

