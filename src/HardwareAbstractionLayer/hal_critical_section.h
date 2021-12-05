#pragma once

#include <PeripheralDriverLayer/pd_timerInterrupt.h>

namespace hal{
    inline void enterCriticalSection(){
        peripheral_driver::stopCMT0();
        peripheral_driver::stopCMT1();
    };

    inline void leaveCriticalSection(){        
        peripheral_driver::startCMT0();
    };
}
