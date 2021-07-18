#pragma once

#include "baseModule.h"
#include "pidController.h"

namespace module{
    class Heater : public BaseModule<Heater>{
      public:
        void update0();
        float getDuty();
        void setTargetTemp(float temp);

      private:
        PidfController _pidf;
        float _p;
        float _i;
        float _i_limit;
        float _limit;
        float _target_temp;
        float _duty;      
        float _temp;
        friend class BaseModule<Heater>;
        Heater();
    };
}
