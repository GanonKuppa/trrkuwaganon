#pragma once

#include "baseModule.h"

namespace module{
    class Heater : public BaseModule<Heater>{
      public:
        void update0();
        void getDuty();
        void setTargetTemp(float temp);

      private:
        float _target_temp;
        float _duty;

        friend class BaseModule<Heater>;
        Heater();
    }
}