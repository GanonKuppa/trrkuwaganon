#pragma once

namespace hal {
    void initPWM0();
    void initPWM1();
    void initPWM2();
    void initPWM3();
    void initPWM4();

    void setDutyPWM0(float duty);
    void setDutyPWM1(float duty);
    void setDutyPWM2(float duty);
    void setDutyPWM3(float duty);
    void setDutyPWM4(float duty);

    float getDutyPWM0();
    float getDutyPWM1();
    float getDutyPWM2();
    float getDutyPWM3();
    float getDutyPWM4();

}