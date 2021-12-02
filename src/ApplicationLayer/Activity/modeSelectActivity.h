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
        ModeSelectActivity();
        virtual std::string getModeName();
        virtual void onStart();
        virtual void onFinish();
        virtual ELoopStatus loop();
      protected:
        uint8_t _mode;
        uint8_t _mode_num;        
        void _turnFcled(uint8_t mode, bool able_goal);
        EActivityColor _modeNum2Color(uint8_t mode);
    };


}
