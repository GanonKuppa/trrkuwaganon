#pragma once

#include "pd_timerInterrupt.h"

namespace hal{
    inline void enterCriticalSection(){
        periferal_driver::stopCMT0();
    };

    inline void leaveCriticalSection(){        
        periferal_driver::startCMT0();
    };
}
