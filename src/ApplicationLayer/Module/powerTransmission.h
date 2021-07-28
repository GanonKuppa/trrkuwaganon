#pragma once

#include "baseModule.h"
#include <stdint.h>

namespace module {
    class PowerTransmission : public BaseModule<PowerTransmission> {
      public:
      	//void update0();                        
        void debug();
        void setDutyL(float duty);
        void setDutyR(float duty);
      private:

        float _duty_r;
        float _duty_l;

        PowerTransmission();
        void _publish();
        

        friend class BaseModule<PowerTransmission>;
    };

    int usrcmd_powerTransmission(int argc, char **argv);
}
