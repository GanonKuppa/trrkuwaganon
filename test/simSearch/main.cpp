#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <string.h>

//Lib
#include "debugLog.h"

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
    module::ParameterManager& pm = module::ParameterManager::getInstance();
    constexpr float DEG2RAD = 3.14159265f / 180.0f;
    constexpr float RAD2DEG = 180.0f/ 3.14159265f;
    float yawrate_max = pm.spin_yawrate_max * DEG2RAD;
    float yawacc = pm.spin_yawacc * DEG2RAD;
    float v = pm.v_search_run;
    float a = pm.a_search_run;

    module::TrajectoryCommander& trajCommander = module::TrajectoryCommander::getInstance();
    module::TrajectoryInitializer::getInstance();
    trajCommander.reset(0.045f, 0.045f, 90.0f*DEG2RAD);

    sim::initSimConnection();
    sim::setReload();
    hal::waitmsec(4000);

    Maze maze;
    Maze maze_gt;    
    maze_archive::setMaze(maze_gt, maze_archive::EMazeName::AllJapan2008Final_HF);
    sim::setWallsWithoutOuter32(maze_gt.walls_vertical, maze_gt.walls_horizontal, maze.walls_vertical, maze.walls_horizontal);

    maze.init();
    maze.writeWall(0, 0, maze_gt.readWall(0, 0));
    maze.writeReached(0, 0, true);

    uint64_t tick_count = 0;
    double real_time = 0.0;
    uint16_t coor_x = 0;
    uint16_t coor_y = 0;
    CtrlSetpointMsg setp_msg;

    StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f, 0.0f, v, v, a, a);
    constexpr double delta_t = 0.001f;

    PRINTF_ASYNC("=========== 1st run ============\n");
    while(real_time < 600.0f) {
        if(real_time > 30.0f && (coor_x == 0 && coor_y == 0)){
            StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f, v, v, 0.0f, a, a);  
            break;
        }

        trajCommander.update0();
        
        copyMsg(msg_id::CTRL_SETPOINT, &setp_msg);
        coor_x = (int)(setp_msg.x/0.09f);
        coor_y = (int)(setp_msg.y/0.09f);

        real_time += delta_t;
        hal::waitmsec(1);
        if(tick_count % 30 == 0) {
            sim::setRobotPos(setp_msg.x, setp_msg.y, setp_msg.yaw*RAD2DEG, setp_msg.v_xy_body);
            sim::addPointRobotContrail(setp_msg.x, setp_msg.y, setp_msg.yaw*RAD2DEG, setp_msg.v_xy_body);
        }
        
        if(setp_msg.traj_type == ETrajType::NONE) {
            uint8_t watch_x = (int)((setp_msg.x + 0.045f * cosf(setp_msg.yaw))/0.09f);
            uint8_t watch_y = (int)((setp_msg.y + 0.045f * sinf(setp_msg.yaw))/0.09f);
            EAzimuth watch_dir = yaw2Azimuth(setp_msg.yaw);
            maze.writeWall(watch_x, watch_y, maze_gt.readWall(watch_x, watch_y));
            maze.writeReached(watch_x, watch_y, true);
            sim::setWallsWithoutOuter32(maze_gt.walls_vertical, maze_gt.walls_horizontal, maze.walls_vertical, maze.walls_horizontal);
    

            int8_t rot_times = 0;
            if(!maze.isReached(maze_gt.goal_x, maze_gt.goal_y)) {
                maze.makeSearchMap(maze_gt.goal_x, maze_gt.goal_y);
                rot_times = maze.calcRotTimes(maze.getSearchDirection2(watch_x, watch_y, watch_dir), watch_dir);
            }
            else{
                maze.makeSearchMap(0, 0);
                rot_times = maze.calcRotTimes(maze.getSearchDirection2(watch_x, watch_y, watch_dir), watch_dir);
            }

            if(rot_times == 0) {
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.09f, v);
            }
            else if(rot_times == 4) {
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f, v, v, 0.0f, a, a);                      
                SpinTurnFactory::push(180.0f * DEG2RAD, yawrate_max, yawacc);
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f, 0.0f, v, v, a, a);
                 
            }
            else if(rot_times == 2) {
                CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, ETurnDir::CCW);                
            }
            else if(rot_times == -2) {
                CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, ETurnDir::CW);
            }
        }
        
        tick_count++;
    }

    while(setp_msg.traj_type != ETrajType::NONE){
        trajCommander.update0();
        real_time += delta_t;
        hal::waitmsec(1);
    }
    PRINTF_ASYNC("1st run end time:%f\n", real_time);

    return 0;
}
