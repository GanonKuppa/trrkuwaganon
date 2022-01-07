#include "hal_gpio.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_gpio.h>
#endif

namespace hal {

    void initGpio() {
#ifndef SILS
        peripheral_driver::initGpio();
        setDout0(false);
        setDout1(false);
        setDout2(false);
        setDout3(false);
        setDout4(false);
        setDout5(false);
        setDout6(false);
        setDout7(false);
#endif
    }

    void setDout0(bool out) {
#ifndef SILS
        peripheral_driver::setDoutP22(out);
#endif
    }

    void setDout1(bool out) {
#ifndef SILS
        peripheral_driver::setDoutP21(out);
#endif
    }

    void setDout2(bool out) {
#ifndef SILS
        peripheral_driver::setDoutP20(out);
#endif
    }

    void setDout3(bool out) {
#ifndef SILS
        peripheral_driver::setDoutP44(out);
#endif
    }

    void setDout4(bool out) {
#ifndef SILS
        peripheral_driver::setDoutP43(out);
#endif
    }

    void setDout5(bool out) {
#ifndef SILS
        peripheral_driver::setDoutP46(out);
#endif
    }

    void setDout6(bool out) {

    }

    void setDout7(bool out) {

    }

}

