#pragma once

namespace periferal_driver {

    void initMTU0();
    void initMTU3();
    void initMTU4();
    void initMTU7();
    void initTPU3();
    void setDutyMTU0(float duty);
    void setDutyMTU3(float duty);
    void setDutyMTU4(float duty);
    void setDutyMTU7(float duty);
    void setDutyTPU3(float duty);

    float getDutyMTU0();
    float getDutyMTU3();
    float getDutyMTU4();
    float getDutyMTU7();
    float getDutyTPU3();
}