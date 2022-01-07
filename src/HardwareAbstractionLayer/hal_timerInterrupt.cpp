#include <stdint.h>

#include "hal_timerInterrupt.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_timerInterrupt.h>
#endif

static float slot0time = 0.0f;
static float slot1time = 0.0f;
static float slot2time = 0.0f;
static float slot3time = 0.0f;

namespace hal {
    void initTimerInterrupt0() {
#ifndef SILS
        peripheral_driver::initCMT0();
#endif
    }

    void setPriorityTimerInterrupt0(uint8_t priori) {
#ifndef SILS
        peripheral_driver::setPriorityCMT0(priori);
#endif
    }

    void startTimerInterrupt0() {
#ifndef SILS
        peripheral_driver::startCMT0();
#endif
    }

    void stopTimerInterrupt0() {
#ifndef SILS
        peripheral_driver::stopCMT0();
#endif
    }

    uint32_t endTimeuCountTimerInterrupt0() {
#ifndef SILS
        return peripheral_driver::endTimeuCountIntCMT0();
#else
        return 0;
#endif
    }

    uint32_t getTimeuCountTimerInterrupt0() {
#ifndef SILS
        return peripheral_driver::getTimeuCountIntCMT0();
#else
        return 0;
#endif

    }

    void setSlot0Time(float usec){slot0time = usec;};
    void setSlot1Time(float usec){slot1time = usec;};
    void setSlot2Time(float usec){slot2time = usec;};
    void setSlot3Time(float usec){slot3time = usec;};

    float getSlot0Time(){return slot0time;};
    float getSlot1Time(){return slot1time;};
    float getSlot2Time(){return slot2time;};
    float getSlot3Time(){return slot3time;};



    void initTimerInterrupt1() {
#ifndef SILS
        peripheral_driver::initCMT1();
#endif
    }

    void setPriorityTimerInterrupt1(uint8_t priori) {
#ifndef SILS
        peripheral_driver::setPriorityCMT1(priori);
#endif
    }

    void startTimerInterrupt1() {
#ifndef SILS
        peripheral_driver::startCMT1();
#endif
    }

    void restartTimerInterrupt1() {
#ifndef SILS
        peripheral_driver::restartCMT1();
#endif
    }


    void stopTimerInterrupt1() {
#ifndef SILS
        peripheral_driver::stopCMT1();
#endif
    }

    void setInterruptPeriodTimerInterrupt1(uint16_t delay_us){
#ifndef SILS
        peripheral_driver::setInterruptPeriodCMT1(delay_us);
#endif
    }


    uint32_t endTimeuCountTimerInterrupt1() {
#ifndef SILS
        return peripheral_driver::endTimeuCountIntCMT1();
#else
        return 0;
#endif

    }

    uint32_t getTimeuCountTimerInterrupt1() {
#ifndef SILS
        return peripheral_driver::getTimeuCountIntCMT1();
#else
        return 0;
#endif
    }

    void enableMultipleinterrupt(){
#ifndef SILS
        __builtin_rx_setpsw('I');
#else
        return;
#endif
        
    }


}


