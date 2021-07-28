#pragma once

namespace periferal_driver {

    void initMTU0();
    void initMTU3();
    void initGPTA1();
    void initGPTA2();
    void initMTU7();
    
    void setDutyMTU0(float duty);
    void setDutyMTU3A(float duty);
    void setDutyMTU3C(float duty);

    void setDutyGPTA1(float duty);
    void setDutyGPTA2(float duty);

    void setDutyMTU7(float duty);    

    float getDutyMTU0();
    float getDutyMTU3A();
    float getDutyMTU3C();
    float getDutyGPTA1();
    float getDutyGPTA2();
    float getDutyMTU7();
    
}