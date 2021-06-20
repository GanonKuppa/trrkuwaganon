#pragma once 

#include "baseModule.h"
#include <deque>

namespace module {
    class BatteryVoltageMonitor : public BaseModule<BatteryVoltageMonitor> {
      public:
        void update();
        void debug();
      private:
        friend class BaseModule<BatteryVoltageMonitor>;
        BatteryVoltageMonitor();
        void _publish();
        float _ad2Voltage(int16_t ad);
        void _lowVoltageCheck();
        float _voltage;
        float _voltage_ave;
        bool _is_low_voltage;
        std::deque<int16_t> _buff;
        uint16_t _count;
        const uint8_t BUFF_SIZE = 10;
        const uint16_t AD_RESOLUTION = 4096;
        const float REG_VOLTAGE = 3.0f;
        const float V_COEF = 30.0f/20.0f;
        const float ALERT_VOL = 3.3f;
    };

}
