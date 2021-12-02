#include "hal_wdt.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_wdt.h>
#endif

namespace hal {
    void resetWdt() {
#ifndef SILS
        peripheral_driver::resetWdt();
#endif
    }

    void initWdt() {
#ifndef SILS
        peripheral_driver::initWdt();
#endif
    }

    void startWdt() {
#ifndef SILS
        peripheral_driver::startWdt();
#endif
    }
}