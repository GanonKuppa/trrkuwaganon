#include "hal_pwm.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_pwm.h>
#endif
namespace hal {
    void initPWM0() {
#ifndef SILS
        peripheral_driver::initMTU0();
#endif
    }

    void initPWM1() {
#ifndef SILS
        peripheral_driver::initMTU3();
#endif
    }

    void initPWM2() {
#ifndef SILS
        peripheral_driver::initMTU3();
#endif
    }

    void initPWM3() {
#ifndef SILS
        peripheral_driver::initGPTA1();
#endif
    }

    void initPWM4() {
#ifndef SILS
        peripheral_driver::initGPTA2();
#endif
    }

    void initPWM5() {
#ifndef SILS
        peripheral_driver::initMTU7();
#endif
    }



    void setDutyPWM0(float duty) {
#ifndef SILS
        peripheral_driver::setDutyMTU0(duty);
#endif
    }

    void setDutyPWM1(float duty) {
#ifndef SILS
        peripheral_driver::setDutyMTU3A(duty);
#endif
    }

    void setDutyPWM2(float duty) {
#ifndef SILS
        peripheral_driver::setDutyMTU3C(duty);
#endif
    }

    void setDutyPWM3(float duty) {
#ifndef SILS
        peripheral_driver::setDutyGPTA1(duty);
#endif
    }

    void setDutyPWM4(float duty) {
#ifndef SILS
        peripheral_driver::setDutyGPTA2(duty);
#endif
    }

    void setDutyPWM5(float duty) {
#ifndef SILS
        peripheral_driver::setDutyMTU7(duty);
#endif
    }



    float getDutyPWM0() {
#ifndef SILS
        return peripheral_driver::getDutyMTU0();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM1() {
#ifndef SILS
        return peripheral_driver::getDutyMTU3A();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM2() {
#ifndef SILS
        return peripheral_driver::getDutyMTU3C();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM3() {
#ifndef SILS
        return peripheral_driver::getDutyGPTA1();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM4() {
#ifndef SILS
        return peripheral_driver::getDutyGPTA2();
#else
        return 0.0f;
#endif
    }

    float getDutyPWM5() {
#ifndef SILS
        return peripheral_driver::getDutyMTU7();
#else
        return 0.0f;
#endif
    }



}
