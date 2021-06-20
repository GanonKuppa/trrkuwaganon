#include <stdint.h>

#include "hal_timerInterrupt.h"

#ifndef SILS
#include "pd_timerInterrupt.h"
#endif

namespace hal {
    void initTimerInterrupt0() {
#ifndef SILS
        periferal_driver::initCMT0();
#endif
    }

    void setPriorityTimerInterrupt0(uint8_t priori) {
#ifndef SILS
        periferal_driver::setPriorityCMT0(priori);
#endif
    }

    void startTimerInterrupt0() {
#ifndef SILS
        periferal_driver::startCMT0();
#endif
    }

    void stopTimerInterrupt0() {
#ifndef SILS
        periferal_driver::stopCMT0();
#endif
    }

    uint32_t endTimeuCountTimerInterrupt0() {
#ifndef SILS
        return periferal_driver::endTimeuCountIntCMT0();
#else
        return 0;
#endif
    }

    uint32_t getTimeuCountTimerInterrupt0() {
#ifndef SILS
        return periferal_driver::getTimeuCountIntCMT0();
#else
        return 0;
#endif

    }

    void initTimerInterrupt1() {
#ifndef SILS
        periferal_driver::initCMT1();
#endif
    }

    void setPriorityTimerInterrupt1(uint8_t priori) {
#ifndef SILS
        periferal_driver::setPriorityCMT1(priori);
#endif
    }

    void startTimerInterrupt1() {
#ifndef SILS
        periferal_driver::startCMT1();
#endif
    }

    void stopTimerInterrupt1() {
#ifndef SILS
        periferal_driver::stopCMT1();
#endif
    }

    uint32_t endTimeuCountTimerInterrupt1() {
#ifndef SILS
        return periferal_driver::endTimeuCountIntCMT1();
#else
        return 0;
#endif

    }

    uint32_t getTimeuCountTimerInterrupt1() {
#ifndef SILS
        return periferal_driver::getTimeuCountIntCMT1();
#else
        return 0;
#endif
    }
}
