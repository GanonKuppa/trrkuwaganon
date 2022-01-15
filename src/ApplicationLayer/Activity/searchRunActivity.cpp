#include "searchRunActivity.h"

// Hal
#include "hal_timer.h"

// Activity
#include "activityFactory.h"

// Module
#include "parameterManager.h"
#include "navigator.h"
#include "positionEstimator.h"
#include "trajectoryCommander.h"
#include "imuDriver.h"
#include "logger.h"
#include "suction.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"
#include "navStateMsg.h"


namespace activity{    
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    std::string SearchRunActivity::getModeName()
    {
        std::string mode_name = "SearchRunActivity";
        return mode_name;
    }

    void SearchRunActivity::onStart(){

    	Intent intent = Intent();
        intent.uint8_t_param["SUB_MODE_NUM"] = 3;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(intent);
        
        intent = activity->getIntent();
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d \n", intent.uint8_t_param["SUB_MODE"]);        
        uint8_t mode = intent.uint8_t_param["SUB_MODE"];
        if(mode == 0) return;
        
        hal::waitmsec(1000);

        float wall2mouse_center_dist = module::ParameterManager::getInstance().wall2mouse_center_dist;
        constexpr float DEG2RAD = 3.14159265f / 180.0f;
        module::ImuDriver::getInstance().calibrateGyro(1000);
        module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
        module::Navigator::getInstance().setNavMode(ENavMode::SEARCH);
        if(mode == 1) module::Navigator::getInstance().setNavSubMode(ENavSubMode::START2GOAL);
        else if(mode == 2) module::Navigator::getInstance().setNavSubMode(ENavSubMode::START2GOAL2START);
        module::Navigator::getInstance().startNavigation();

        module::Logger::getInstance().start();
        module::Suction::getInstance().setDuty(0.4f);
        hal::waitmsec(100);
    }
    
    
    void SearchRunActivity::onFinish(){
        module::Suction::getInstance().setDuty(0.0f);
        module::Navigator::getInstance().endNavigation();
    }

    SearchRunActivity::ELoopStatus SearchRunActivity::loop() {
        CtrlSetpointMsg ctrl_msg;
        NavStateMsg nav_msg;
        copyMsg(msg_id::NAV_STATE, &nav_msg);
        ELoopStatus loop_status = ELoopStatus::CONTINUE;
        
        if(nav_msg.is_failsafe){
            module::TrajectoryCommander::getInstance().clear();
            loop_status = ELoopStatus::FINISH;
        }

        if(!nav_msg.navigating && ctrl_msg.traj_type == ETrajType::NONE){            
            loop_status = ELoopStatus::FINISH;
        }

        return loop_status;
    }
}

