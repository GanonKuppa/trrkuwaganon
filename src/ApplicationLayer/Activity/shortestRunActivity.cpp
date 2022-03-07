#include "shortestRunActivity.h"

#include <vector>

// Hal
#include "hal_timer.h"

// Lib
#include "path.h"
#include "pathCalculation.h"

// Activity
#include "activityFactory.h"
#include "intent.h"

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
#include "turnEnum.h"
#include "maze.h"
#include "trajectoryFactory.h"


namespace activity{    


    std::string ShortestRunActivity::getModeName()
    {
        std::string mode_name = "ShortestRunActivity";
        return mode_name;
    }

    void ShortestRunActivity::onStart(){        
        uint8_t param_mode = 0;

        Intent intent = Intent();
        intent.uint8_t_param["SUB_MODE_NUM"] = 8;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(intent);
        
        intent = activity->getIntent();
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d \n", intent.uint8_t_param["SUB_MODE"]);        
        param_mode = intent.uint8_t_param["SUB_MODE"];
        if(param_mode == 0) return;


        hal::waitmsec(100);

        float wall2mouse_center_dist = module::ParameterManager::getInstance().wall2mouse_center_dist;
        constexpr float DEG2RAD = 3.14159265f / 180.0f;
        module::ImuDriver::getInstance().calibrateGyro(1000);
        module::ImuDriver::getInstance().calibrateAcc(1000);
        module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
        module::Navigator::getInstance().setNavMode(ENavMode::FASTEST);
                
        ETurnParamSet tp = ETurnParamSet(param_mode);
        module::ParameterManager& pm = module::ParameterManager::getInstance();
        std::vector<Path> path_vec;
        Maze& maze = module::Navigator::getInstance().getMazeRef();

        makeFastestDiagonalPath(500, tp, pm.goal_x, pm.goal_y, maze, path_vec);

        StopFactory::push(2.1f);
        float suction_duty = module::ParameterManager::getInstance().suction_duty_shortest;
        module::Suction::getInstance().setDuty(suction_duty);
        CtrlSetpointMsg ctrl_msg;
        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        HF_playPath(tp, path_vec);        
        while(ctrl_msg.traj_type == ETrajType::STOP || ctrl_msg.traj_type == ETrajType::NONE){
        	copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        	hal::waitmsec(1);
        }
        module::Logger::getInstance().start();

        PRINTF_ASYNC("--- makeMinStepPath ----\n");
        printPath(path_vec);
        
        hal::waitmsec(100);

    }
    
    
    void ShortestRunActivity::onFinish(){
        module::Suction::getInstance().setDuty(0.0f);
        module::Logger::getInstance().end();
    }


    ShortestRunActivity::ELoopStatus ShortestRunActivity::loop() {
        CtrlSetpointMsg ctrl_msg;
        NavStateMsg nav_msg;
        copyMsg(msg_id::NAV_STATE, &nav_msg);
        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        ELoopStatus loop_status = ELoopStatus::CONTINUE;
        
        if(nav_msg.is_failsafe){
            module::TrajectoryCommander::getInstance().clear();
            loop_status = ELoopStatus::FINISH;
            module::Suction::getInstance().setDuty(0.0f);
        }

        if(ctrl_msg.traj_type == ETrajType::NONE){            
            loop_status = ELoopStatus::FINISH;
            module::Suction::getInstance().setDuty(0.0f);
        }

        return loop_status;
    }
}

