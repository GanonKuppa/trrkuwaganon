#include "hal_da.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_da.h>
#endif

namespace hal {
    void initDA() {
#ifndef SILS
        peripheral_driver::initDA();
#endif
    }

    void setDA(float da) {
#ifndef SILS
        peripheral_driver::setDA(da * (DA_RESOLUTION - 1));
#endif
    }

    float getDA() {
#ifndef SILS
        return peripheral_driver::getDA() / (DA_RESOLUTION - 1);
#else
        return 0;
#endif
    }

}
