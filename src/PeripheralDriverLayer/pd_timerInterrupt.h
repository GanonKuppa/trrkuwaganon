#pragma once
#include <stdint.h>

namespace peripheral_driver {
    void initCMT0();
    void setPriorityCMT0(uint8_t priori);
    void startCMT0();
    void stopCMT0();
    void setEnableCMT0(bool enable);

    uint32_t endTimeuCountIntCMT0();
    uint32_t getTimeuCountIntCMT0();

    void initCMT1();
    void setPriorityCMT1(uint8_t priori);
    void restartCMT1();
    void startCMT1();
    void stopCMT1();

    void setInterruptPeriodCMT1(uint16_t delay_us);

    uint32_t endTimeuCountIntCMT1();
    uint32_t getTimeuCountIntCMT1();
}

