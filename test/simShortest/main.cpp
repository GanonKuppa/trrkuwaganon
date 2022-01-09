#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <string.h>
#include <vector>

//Lib
#include "debugLog.h"
#include "path.h"
#include "pathCalculation.h"

// Hal
#include "hal_timer.h"

// Module
#include "parameterManager.h"
#include "trajectoryInitializer.h"
#include "trajectoryCommander.h"

// Object
#include "maze.h"
#include "mazeArchive.h"
#include "trajectoryFactory.h"
#include "turnEnum.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"

// SilsUtil
#include "sendData2Sim.h"

int main() {        
    constexpr float DEG2RAD = 3.14159265f / 180.0f;
    constexpr float RAD2DEG = 180.0f/ 3.14159265f;

    module::ParameterManager& pm = module::ParameterManager::getInstance();
    module::TrajectoryCommander& trajCommander = module::TrajectoryCommander::getInstance();
    module::TrajectoryInitializer::getInstance();

    sim::initSimConnection();
    sim::setReload();
    hal::waitmsec(4000);

    Maze maze;
    maze_archive::setMaze(maze, maze_archive::EMazeName::AllJapan2008Final_HF);
    for(uint16_t i=0;i<32;i++){
        for(uint16_t j=0;j<32;j++) maze.writeReached(i, j, true);
    }
    sim::setWallsWithoutOuter32(maze.walls_vertical, maze.walls_horizontal);    

    
    uint64_t tick_count = 0;
    double real_time = 0.0;
    CtrlSetpointMsg setp_msg;
    constexpr double delta_t = 0.001f;

    float wall2mouse_center_dist = pm.wall2mouse_center_dist;
    trajCommander.reset(0.045f, 0.045f - wall2mouse_center_dist, 90.0f * DEG2RAD);
    
    ETurnParamSet tp = ETurnParamSet(7);
    std::vector<Path> path_vec;        

    makeFastestDiagonalPath(500, tp, maze.goal_x, maze.goal_y, maze, path_vec);
    HF_playPath(tp, path_vec);
    PRINTF_ASYNC("--- makeMinStepPath ----\n");
    printPath(path_vec);

    PRINTF_ASYNC("=========== shortest run ============\n");
    while(setp_msg.traj_type != ETrajType::NONE){                
        trajCommander.update0();
        copyMsg(msg_id::CTRL_SETPOINT, &setp_msg);
        real_time += delta_t;
        hal::waitmsec(1);
        if(tick_count % 30 == 0) {
            sim::setRobotPos(setp_msg.x, setp_msg.y, setp_msg.yaw*RAD2DEG, setp_msg.v_xy_body);
            sim::addPointRobotContrail(setp_msg.x, setp_msg.y, setp_msg.yaw*RAD2DEG, setp_msg.v_xy_body);
        }
        //PRINTF_ASYNC("%f, %f, %d, %d\n", setp_msg.x, setp_msg.y, (uint8_t)setp_msg.traj_type, (uint8_t)setp_msg.turn_type);
        tick_count++;        
    }
    PRINTF_ASYNC("shortest run end time:%f\n", real_time);

    return 0;
}

