#pragma once

#include "baseModule.h"
#include "led.h"

#include <deque>
#include <stdint.h>

namespace module {
    class WallSensor : public BaseModule<WallSensor> {
      public:
        void update(){};
    	void update1();
        void update2();        
        void setEnable(bool en);
        void debug();
      private:
        bool _enable;
        int16_t _ahead_l_on;
        int16_t _ahead_r_on;
        int16_t _left_on;
        int16_t _right_on;

        int16_t _ahead_l_off;
        int16_t _ahead_r_off;
        int16_t _left_off;
        int16_t _right_off;

        std::deque<int16_t> _ahead_l_q;
        std::deque<int16_t> _ahead_r_q;
        std::deque<int16_t> _left_q;
        std::deque<int16_t> _right_q;


        const uint8_t BUFF_SIZE = 30;
        const uint16_t LED_ON_USEC = 30;

        WallSensor();
        void _updateOffVal(bool sled1, bool sled2, bool sled3, bool sled4);
        void _updateOnVal(bool sled1, bool sled2, bool sled3, bool sled4);
        void _turnLed(bool sled1, bool sled2, bool sled3, bool sled4);
        void _modulateVal();
        void _publish();

        friend class BaseModule<WallSensor>;
    };
}
