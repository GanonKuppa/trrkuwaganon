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

// Obj
#include "trajectoryFactory.h"


namespace activity{    
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    std::string SearchRunActivity::getModeName()
    {
        std::string mode_name = "SearchRunActivity";
        return mode_name;
    }

    void SearchRunActivity::onStart(){

    	Intent intent = Intent();
        intent.uint8_t_param["SUB_MODE_NUM"] = 4;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(intent);
        
        intent = activity->getIntent();
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d \n", intent.uint8_t_param["SUB_MODE"]);        
        uint8_t mode = intent.uint8_t_param["SUB_MODE"];
        if(mode == 0) return;
        
        hal::waitmsec(100);

        float wall2mouse_center_dist = module::ParameterManager::getInstance().wall2mouse_center_dist;
        constexpr float DEG2RAD = 3.14159265f / 180.0f;
        module::ImuDriver::getInstance().calibrateGyro(1000);
        module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
        module::Navigator::getInstance().setNavMode(ENavMode::SEARCH);
        if(mode == 1) module::Navigator::getInstance().setNavSubMode(ENavSubMode::START2GOAL);
        else if(mode == 2) module::Navigator::getInstance().setNavSubMode(ENavSubMode::START2GOAL2START);
        else if(mode == 3) module::Navigator::getInstance().setNavSubMode(ENavSubMode::ALL_AREA_SEARCH);        
        float suction_duty = module::ParameterManager::getInstance().suction_duty_search;
        module::Suction::getInstance().setDuty(suction_duty);
        hal::waitmsec(2000);
        module::Navigator::getInstance().startNavigation();

        module::Logger::getInstance().start();
        hal::waitmsec(100);
    }
    
    
    void SearchRunActivity::onFinish(){                
        module::Navigator::getInstance().endNavigation();
        module::Logger::getInstance().end();
    }

    SearchRunActivity::ELoopStatus SearchRunActivity::loop() {
        CtrlSetpointMsg ctrl_msg;
        NavStateMsg nav_msg;
        copyMsg(msg_id::NAV_STATE, &nav_msg);
        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        ELoopStatus loop_status = ELoopStatus::CONTINUE;
        


        if((!nav_msg.navigating && ctrl_msg.traj_type == ETrajType::NONE )|| nav_msg.is_failsafe){ 
            module::TrajectoryCommander::getInstance().clear();
            module::Suction::getInstance().setDuty(0.0f);
            loop_status = ELoopStatus::FINISH;
        }

        return loop_status;
    }
}

