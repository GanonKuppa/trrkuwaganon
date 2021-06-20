#include "hal_wdt.h"

#ifndef SILS
#include "pd_wdt.h"
#endif

namespace hal {
    void resetWdt() {
#ifndef SILS
        periferal_driver::resetWdt();
#endif
    }

    void initWdt() {
#ifndef SILS
        periferal_driver::initWdt();
#endif
    }

    void startWdt() {
#ifndef SILS
        periferal_driver::startWdt();
#endif
    }
}