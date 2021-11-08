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
        //mode = intent.uint8_t_param["SUB_MODE"];

        module::Navigator& nav = module::Navigator::getInstance();
        nav.setNavMode(ENavMode::STANDBY);
        nav.setNavSubMode(ENavSubMode::STANDBY);

        module::ParameterManager& pm = module::ParameterManager::getInstance();
        constexpr float DEG2RAD = 3.14159265f / 180.0f;
        float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
        float yawacc = pm.spin_yawacc * DEG2RAD;

        module::ImuDriver::getInstance().calibrateGyro(1000);
        module::TrajectoryCommander::getInstance().reset(0.09f, 0.09f, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.09f, 0.09f, 90.0f * DEG2RAD);
        

        //SpinTurnFactory::push(90.0f * DEG2RAD, yawrate_max, yawacc);
        StopFactory::push(20.0f);
        //SpinTurnFactory::push(-90.0f * DEG2RAD, yawrate_max, yawacc);
        StopFactory::push(2.0f);
        //SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
        StopFactory::push(2.0f);
        //SpinTurnFactory::push(-180.0f * DEG2RAD, yawrate_max, yawacc);
        StopFactory::push(2.0f);
        
        module::Logger::getInstance().start();
        PRINTF_ASYNC("TRAJ PUSH END\n");
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

