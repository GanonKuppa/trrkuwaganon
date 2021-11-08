#include "debugActivity.h"

#include <memory>

#include "debugLog.h"

#include "activityFactory.h"
#include "intent.h"

#include "msgBroker.h"

#include "ctrlSetpointMsg.h"

#include "trajectoryFactory.h"
#include "trajectoryCommander.h"
#include "parameterManager.h"
#include "navigator.h"
#include "positionEstimator.h"
#include "imuDriver.h"
#include "logger.h"

#include "hal_timer.h"

namespace activity{    


    

    std::string DebugActivity::getModeName()
    {
        std::string mode_name = "DebugActivity";
        return mode_name;
    }

    void DebugActivity::onStart(){
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
        module::TrajectoryCommander::getInstance().reset(0.09f, 0.09f, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.09f, 0.09f, 90.0f * DEG2RAD);
        
        if(mode == 1){
            StopFactory::push(20.0f);
        }
        else if(mode == 2){
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;
            SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(-90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(-180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);
        }
        else if(mode == 3){
            ETurnType turn_type;
            if(pm.test_run_wall_flag == 1){
                turn_type = ETurnType::STRAIGHT_CENTER;
            }
            else{
                turn_type = ETurnType::STRAIGHT;
            }

            float target_dist = pm.test_run_x;
            float v_0 = 0.0f;
            float v_max = pm.test_run_v;
            float v_end = 0.0f;
            float a_acc = pm.test_run_a;
            float a_dec = pm.test_run_a;
            StraightFactory::push(turn_type, target_dist, v_0, v_max, v_end, a_acc, a_dec);
            StopFactory::push(2.0f);
        }
        else if(mode == 4){
        	ETurnType turn_type = ETurnType::STRAIGHT_CENTER;
        	float target_dist = 9.0 * 8.0f;
            float v_0 = 0.0f;
            float v_max = pm.v_search_run;
            float v_end = 0.0f;
            float a_acc = pm.a_search_run;
            float a_dec = pm.a_search_run;
            StraightFactory::push(turn_type, target_dist, v_0, v_max, v_end, a_acc, a_dec);
            StopFactory::push(2.0f);
        }
        else if(mode == 5){

        }
        else if(mode == 6){

        }
        else if(mode == 7){
            
        }
        
        module::Logger::getInstance().start();        
        hal::waitmsec(100);
    }
    
    
    void DebugActivity::onFinish(){ }


    DebugActivity::ELoopStatus DebugActivity::loop() {
        CtrlSetpointMsg ctrl_msg;
        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        ELoopStatus loop_status = ELoopStatus::CONTINUE;  
        
        if(ctrl_msg.traj_type == ETrajType::NONE){
            loop_status = ELoopStatus::FINISH;
        }

        return loop_status;
    }
}

