#pragma once


#include <stdint.h>

#include "baseActivity.h"


namespace activity {

    class RadioControlActivity : public BaseActivity {
      public:
        std::string getModeName();
        void onStart();
        void onFinish();
        ELoopStatus loop();
    };


}

