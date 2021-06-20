#pragma once


#include <stdint.h>

#include "baseActivity.h"
#include "activityFactory.h"

namespace activity {

    class SubModeSelectActivity : public BaseActivity {
      public:
        std::string getModeName();
        void onStart();
        void onFinish();
        ELoopStatus loop();
    };


}
