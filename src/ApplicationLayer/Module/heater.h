#pragma once

#include "baseModule.h"

namespace module{
    class Heater : public BaseModule<Heater>{
      public:
        void update();
        void getDuty();
        void setTargetTemp(float temp);

      private:
        float _target_temp;
        float _duty;

        friend class BaseModule<Heater>;
        Heater();
    }
}