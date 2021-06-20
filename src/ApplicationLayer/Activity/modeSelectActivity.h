#pragma once

#include <stdint.h>
#include <string>
#include "activityFactory.h"
#include "baseActivity.h"
#include "gamepadMsg.h"

namespace activity {

    class ModeSelectActivity : public BaseActivity {
      public:
        std::string getModeName();
        void onStart();
        void onFinish();
        ELoopStatus loop();
      private:
        uint8_t _mode;
        GamepadMsg _gp_msg;

        static constexpr uint8_t MODE_NUM = 8;
        void turnFcled();
        EActivityColor modeNum2Color(uint8_t mode);
    };


}