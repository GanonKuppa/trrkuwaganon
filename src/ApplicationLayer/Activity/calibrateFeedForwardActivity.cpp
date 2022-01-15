#include "calibrateFeedForwardActivity.h"

#include <memory>

// Lib
#include "debugLog.h"

// Hal
#include "hal_timer.h"

// Activity
#include "activityFactory.h"
#include "intent.h"

// Module
#include "trajectoryCommander.h"
#include "parameterManager.h"
#include "navigator.h"
#include "positionEstimator.h"
#include "imuDriver.h"
#include "logger.h"

#include "ledController.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"
#include "navStateMsg.h"

// Obj
#include "trajectoryFactory.h"
#include "turnEnum.h"


namespace activity{    


    std::string CalibrateFeedForwardActivity::getModeName()
    {
        std::string mode_name = "CalibrateFeedForwardActivity";
        return mode_name;
    }

    void CalibrateFeedForwardActivity::onStart(){
        Intent intent = Intent();
        intent.uint8_t_param["SUB_MODE_NUM"] = 8;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(intent);
        
        intent = activity->getIntent();
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d \n", intent.uint8_t_param["SUB_MODE"]);        
        uint8_t mode = intent.uint8_t_param["SUB_MODE"];
        if(mode == 0) return;

        module::Navigator& nav = module::Navigator::getInstance();
        nav.setNavMode(ENavMode::STANDBY);
        nav.setNavSubMode(ENavSubMode::STANDBY);

        module::ParameterManager& pm = module::ParameterManager::getInstance();
        constexpr float DEG2RAD = 3.14159265f / 180.0f;

        hal::waitmsec(1000);
        module::ImuDriver::getInstance().calibrateGyro(1000);
        module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);


        if(mode == 1){
            float v_0 = 0.0f;
            float v_max = 0.1;
            float a = 5.0f;
            float x = 6.0f * 0.09f;
            float stop_time = 0.3f;
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;
            uint16_t run_num = 20;

            for(int i=0;i<run_num;i++){
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, x, v_0, v_max, 0.0f, a, a);
                StopFactory::push(stop_time);
                SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
                StopFactory::push(stop_time);
                v_max += 0.1;
            }            
        }
        else if(mode == 2){
            float v_0 = 0.0f;
            float v_max = 1.0f;
            float a = 1.0f;
            float x = 6.0f * 0.09f;
            float stop_time = 0.3f;
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;
            uint16_t run_num = 30;

            for(int i=0;i<run_num;i++){
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, x, v_0, v_max, 0.0f, a, a);
                StopFactory::push(stop_time);
                SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
                StopFactory::push(stop_time);
                a += 0.1;
            }
        }
        else if(mode == 3){
            float stop_time = 0.3f;
            float yawrate_max = 500.0f * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;
            uint16_t run_num = 10;

            for(int i=0;i<run_num;i++){
                StopFactory::push(stop_time);
                SpinTurnFactory::push(720.0f * DEG2RAD, yawrate_max, yawacc);
                StopFactory::push(stop_time);
                yawrate_max += 100.0f * DEG2RAD;
            }
        }
        else if(mode == 4){
            float stop_time = 0.3f;
            float yawrate_max = 5000.0f * DEG2RAD;
            float yawacc = pm.spin_yawacc = 500.0f * DEG2RAD;
            uint16_t run_num = 30;

            for(int i=0;i<run_num;i++){
                StopFactory::push(stop_time);
                SpinTurnFactory::push(360.0f * DEG2RAD, yawrate_max, yawacc);
                StopFactory::push(stop_time);
                yawacc += 500.0f * DEG2RAD;
            }
        }
        else if(mode == 5){
        }
        else if(mode == 6){        	
        }
        else if(mode == 7){
        }
        
        module::Logger::getInstance().start();        
        module::LedController::getInstance().flashFcled(1,1,1, 0.5, 0.5);
        hal::waitmsec(100);
    }
    
    
    void CalibrateFeedForwardActivity::onFinish(){
        module::Logger::getInstance().end();
    }


    CalibrateFeedForwardActivity::ELoopStatus CalibrateFeedForwardActivity::loop() {
        CtrlSetpointMsg ctrl_msg;
        NavStateMsg nav_msg;

        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        copyMsg(msg_id::NAV_STATE, &nav_msg);

        ELoopStatus loop_status = ELoopStatus::CONTINUE;

        
        if(ctrl_msg.traj_type == ETrajType::NONE || nav_msg.is_failsafe){
            module::TrajectoryCommander::getInstance().clear();
            loop_status = ELoopStatus::FINISH;
        }

        return loop_status;        
    }
}
