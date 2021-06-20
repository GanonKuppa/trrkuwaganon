#include "hal_pwm.h"

#ifndef SILS
#include "pd_pwm.h"
#endif
namespace hal {
    void initPWM0() {
#ifndef SILS
        periferal_driver::initMTU0();
#endif
    }

    void initPWM1() {
#ifndef SILS
        periferal_driver::initMTU3();
#endif
    }

    void initPWM2() {
#ifndef SILS
        periferal_driver::initMTU4();
#endif
    }

    void initPWM3() {
#ifndef SILS
        periferal_driver::initTPU3();
#endif
    }

    void initPWM4() {
#ifndef SILS
        periferal_driver::initMTU7();
#endif
    }


    void setDutyPWM0(float duty) {
#ifndef SILS
        periferal_driver::setDutyMTU0(duty);
#endif
    }

    void setDutyPWM1(float duty) {
#ifndef SILS
        periferal_driver::setDutyMTU3(duty);
#endif
    }

    void setDutyPWM2(float duty) {
#ifndef SILS
        periferal_driver::setDutyMTU4(duty);
#endif
    }

    void setDutyPWM3(float duty) {
#ifndef SILS
        periferal_driver::setDutyTPU3(duty);
#endif
    }

    void setDutyPWM4(float duty) {
#ifndef SILS
        periferal_driver::setDutyMTU7(duty);
#endif
    }


    float getDutyPWM0() {
#ifndef SILS
        return periferal_driver::getDutyMTU0();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM1() {
#ifndef SILS
        return periferal_driver::getDutyMTU3();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM2() {
#ifndef SILS
        return periferal_driver::getDutyMTU4();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM3() {
#ifndef SILS
        return periferal_driver::getDutyTPU3();
#else
        return 0.0f;
#endif

    }

    float getDutyPWM4() {
#ifndef SILS
        return periferal_driver::getDutyMTU7();
#else
        return 0.0f;
#endif
    }


}