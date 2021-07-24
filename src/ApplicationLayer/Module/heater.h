#pragma once

#include "baseModule.h"
#include "pidController.h"

namespace module{
    class Heater : public BaseModule<Heater>{
      public:
        void update0();
        float getDuty();
        void setTargetTemp(float temp);
        void debug();
        void eval(uint32_t num);

      private:
        PidfController _pidf;
        float _p;
        float _i;
        float _i_limit;
        float _limit;
        float _target_temp;
        float _duty;      
        float _temp;        
        const float _resistor_ohm = 6.8f * 2.0f;
        float _current_ave;
        float _current_1sec_sum;
        float _current_time;
        float _voltage;
        friend class BaseModule<Heater>;
        Heater();
    };

    int usrcmd_heater(int argc, char **argv);
}
