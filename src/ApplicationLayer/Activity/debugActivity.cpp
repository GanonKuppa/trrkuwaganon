#include "debugActivity.h"

#include <memory>

#include "activityFactory.h"
#include "intent.h"

#include "debugLog.h"

namespace activity{    


    

    std::string DebugActivity::getModeName()
    {
        std::string mode_name = "DebugActivity";
        return mode_name;
    }

    void DebugActivity::onStart(){
        std::unique_ptr<Intent> intent = std::make_unique<Intent>();
        intent->uint8_t_param["SUB_MODE_NUM"] = 8;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(std::move(intent));
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d", intent->uint8_t_param["SUB_MODE"]);
        //mode = intent->uint8_t_param["SUB_MODE"];

    }
    
    
    void DebugActivity::onFinish(){

    }


    DebugActivity::ELoopStatus DebugActivity::loop() {
        return ELoopStatus::FINISH;
    }
}

