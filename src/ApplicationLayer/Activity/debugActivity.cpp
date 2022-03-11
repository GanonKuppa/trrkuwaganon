#include "debugActivity.h"

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
#include "trajectoryInitializer.h"
#include "parameterManager.h"
#include "navigator.h"
#include "positionEstimator.h"
#include "imuDriver.h"
#include "logger.h"
#include "suction.h"

#include "ledController.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"
#include "navStateMsg.h"

// Obj
#include "trajectoryFactory.h"
#include "turnEnum.h"


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
        module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);

        float suction_duty = module::ParameterManager::getInstance().suction_duty_search;
        module::Suction::getInstance().setDuty(suction_duty);
        hal::waitmsec(2000);

        
        if(mode == 1){
            StopFactory::push(20.0f);
        }
        else if(mode == 2){
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;
            SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(1.0f);
            SpinTurnFactory::push(-90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(1.0f);
            SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(1.0f);
            SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(1.0f);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(0.5f);
            SpinTurnFactory::push(-180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(0.5f);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(0.5f);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(0.5f);
        }
        else if(mode == 3){
            ETurnType turn_type;
            if(pm.test_run_wall_flag == 1){
                turn_type = ETurnType::STRAIGHT_CENTER;
            }
            else{
                turn_type = ETurnType::STRAIGHT;
            }

            module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
            module::PositionEstimator::getInstance().reset(0.045f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
        	float target_dist = 0.09 * 2.0f + pm.wall2mouse_center_dist * 2 - 0.002f;          
            float v_0 = 0.0f;
            float v_max = 0.1f;
            float v_end = 0.0f;
            float a_acc = 1.0f;
            float a_dec = 1.0f;
            StraightFactory::push(turn_type, target_dist, v_0, v_max, v_end, a_acc, a_dec);
            StopFactory::push(2.0f);
        }
        else if(mode == 4){
        	
            ETurnType turn_type = ETurnType::STRAIGHT_CENTER;
            module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
            module::PositionEstimator::getInstance().reset(0.045f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
        	float target_dist = 0.09 * 31.0f + pm.wall2mouse_center_dist;
            float v_0 = 0.0f;
            float v_max = 0.3f;
            float v_end = 0.0f;
            float a_acc = 0.5f;
            float a_dec = 0.5f;
            StraightFactory::push(turn_type, target_dist, v_0, v_max, v_end, a_acc, a_dec);
            StopFactory::push(2.0f);
        }
        else if(mode == 5){
            module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
            module::PositionEstimator::getInstance().reset(0.045f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
        	float target_dist = 0.045f + 0.09f;
            float v_0 = 0.0f;
            float v_max = module::TrajectoryInitializer::getInstance().getV(ETurnParamSet::SAFE, ETurnType::TURN_90);
            float v_end = 0.0f;
            float a_acc = 4.0f;
            float a_dec = 4.0f;
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;

            ETurnType turn_type = ETurnType::STRAIGHT;
            StraightFactory::push(turn_type, target_dist + pm.wall2mouse_center_dist, v_0, v_max, v_max, a_acc, a_dec);
            CurveFactory::pushWithStraight(ETurnParamSet::SAFE, ETurnType::TURN_90, ETurnDir::CW);
            StraightFactory::push(turn_type, target_dist, v_max, v_max, v_end, a_acc, a_dec);
            StopFactory::push(2.0f);
        }
        else if(mode == 6){        	
            module::TrajectoryCommander::getInstance().reset(0.045f + 0.09f * 2.0f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
            module::PositionEstimator::getInstance().reset(0.045f + 0.09f * 2.0f, 0.045f - pm.wall2mouse_center_dist, 90.0f * DEG2RAD);
        	float target_dist = 0.045 + 0.09f;
            float v_0 = 0.0f;
            float v_max = module::TrajectoryInitializer::getInstance().getV(ETurnParamSet::SAFE, ETurnType::TURN_90);
            float v_end = 0.0f;
            float a_acc = 4.0f;
            float a_dec = 4.0f;
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;

            ETurnType turn_type = ETurnType::STRAIGHT;
            StraightFactory::push(turn_type, target_dist + pm.wall2mouse_center_dist, v_0, v_max, v_max, a_acc, a_dec);
            CurveFactory::pushWithStraight(ETurnParamSet::SAFE, ETurnType::TURN_90, ETurnDir::CCW);
            StraightFactory::push(turn_type, target_dist, v_max, v_max, v_end, a_acc, a_dec);
            StopFactory::push(2.0f);



        }
        else if(mode == 7){
            float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
            float yawacc = pm.spin_yawacc * DEG2RAD;
            AheadWallCorrectionFactory::push(30.0f, true);
            SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
            StopFactory::push(2.0f);

        }
        
        module::Logger::getInstance().start();       
        module::LedController::getInstance().flashFcled(1,1,1, 0.5, 0.5);
        hal::waitmsec(100);
        
    }
    
    
    void DebugActivity::onFinish(){
        module::Logger::getInstance().end();
        module::Suction::getInstance().setDuty(0.0f);
     }


    DebugActivity::ELoopStatus DebugActivity::loop() {
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

