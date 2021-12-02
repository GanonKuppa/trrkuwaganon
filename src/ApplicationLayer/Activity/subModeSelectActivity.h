#pragma once


#include <stdint.h>

#include "modeSelectActivity.h"

namespace activity {

    class SubModeSelectActivity : public ModeSelectActivity {
      public:
        virtual std::string getModeName();
        virtual void onStart();
        virtual void onFinish();
        virtual ELoopStatus loop();
      private:
        float _led_on_sec;
        float _led_off_sec;
        void _turnFcled(uint8_t mode);
    };
}
