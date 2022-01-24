#include "deleteMazeActivity.h"


// Activity
#include "activityFactory.h"

// Hal
#include "hal_timer.h"
#include "hal_critical_section.h"

// Module
#include "navigator.h"
#include "ledController.h"

namespace activity{    


    std::string DeleteMazeActivity::getModeName()
    {
        std::string mode_name = "DeleteMazeActivity";
        return mode_name;
    }

    void DeleteMazeActivity::onStart(){
        Intent intent = Intent();
        intent.uint8_t_param["SUB_MODE_NUM"] = 2;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(intent);
        
        intent = activity->getIntent();
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d \n", intent.uint8_t_param["SUB_MODE"]);        
        uint8_t mode = intent.uint8_t_param["SUB_MODE"];
        if(mode == 0) return;
        
        hal::waitmsec(1000);

        if(mode == 1){
            module::Navigator::getInstance().getMazeRef().init();
            //hal::enterCriticalSection();            
            module::Navigator::getInstance().getMazeRef().writeMazeData2Flash();                
            //hal::leaveCriticalSection();
            
            module::LedController::getInstance().turnFcled(0,0,1);
            hal::waitmsec(150);
            module::LedController::getInstance().turnFcled(0,0,0);
            hal::waitmsec(50);
            module::LedController::getInstance().turnFcled(0,0,1);
            hal::waitmsec(50);
            module::LedController::getInstance().turnFcled(0,0,0);
            hal::waitmsec(50);
        }

    }
    
    
    void DeleteMazeActivity::onFinish(){

    }


    DeleteMazeActivity::ELoopStatus DeleteMazeActivity::loop() {

        return ELoopStatus::FINISH;
    }
}

