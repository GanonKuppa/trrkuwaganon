#pragma once

#include <stdint.h>
#include <string>
#include "activityFactory.h"
#include "baseActivity.h"

// Msg
#include "gamepadMsg.h"
#include "dialPositionMsg.h"

namespace activity {

    class ModeSelectActivity : public BaseActivity {
      public:
        std::string getModeName();
        void onStart();
        void onFinish();
        ELoopStatus loop();
      private:
        uint8_t _mode;
        static constexpr uint8_t MODE_NUM = 8;
        void _turnFcled(uint8_t mode, bool able_goal);
        EActivityColor _modeNum2Color(uint8_t mode);
    };


}
