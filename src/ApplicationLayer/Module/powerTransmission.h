#pragma once

#include "baseModule.h"
#include <stdint.h>

namespace module {
    class PowerTransmission : public BaseModule<PowerTransmission> {
      public:
      	void update0();                        
        void debug();        
        void setDutyR(float duty);
        void setDutyL(float duty);
        void setMaxVoltageDutyR(float duty);
        void setMaxVoltageDutyL(float duty);
      private:

        float _duty_r;
        float _duty_l;
        float _voltage;

        static constexpr float MAX_VOLTAGE = 4.2f;
        
        PowerTransmission();
        void _publish();
        

        friend class BaseModule<PowerTransmission>;
    };

    int usrcmd_powerTransmission(int argc, char **argv);
}
