#include "debugActivity.h"


namespace activity{    


    std::string DebugActivity::getModeName()
    {
        std::string mode_name = "DebugActivity";
        return mode_name;
    }

    void DebugActivity::onStart(){

    }
    
    
    void DebugActivity::onFinish(){

    }


    DebugActivity::ELoopStatus DebugActivity::loop() {

        return ELoopStatus::CONTINUE;
    }
}

