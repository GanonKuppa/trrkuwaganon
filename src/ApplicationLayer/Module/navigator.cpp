#include "navigator.h"

#include <cmath>


// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_timer.h"

// Msg
#include "msgBroker.h"
#include "vehiclePositionMsg.h"
#include "vehicleAttitudeMsg.h"
#include "ctrlSetPointMsg.h"

// Module
#include "parameterManager.h"
#include "trajectoryFactory.h"


namespace module{

    Navigator::Navigator() :
        _mode(ENavMode::MODE_SELECT),
        _sub_mode(ENavSubMode::STANDBY),
        _lock_guard(false),
        _navigating(false),
        _done_outward(false),
        _x_cur(0),
        _y_cur(0),
        _azimuth(EAzimuth::N),
        _x_dest(7),
        _y_dest(7),
        _x_goal(7),
        _y_goal(7),
        _r_wall_enable(false),
        _l_wall_enable(false),
        _is_failsafe(false),
        _in_read_wall_area(false),
        _x(0.045f),
        _y(0.045f),
        _yaw(90.0f * DEG2RAD),
        _v(0.3f),
        _a(3.0f),
        _yawrate_max(1000.0f * DEG2RAD),
        _yawacc(1000.0f * DEG2RAD),
        _wall2mouse_center_dist(0.0f),
        _read_wall_offset1(0.001),
        _read_wall_offset2(0.003)
    {
        setModuleName("Navigator");
    }
    
    void Navigator::setNavMode(ENavMode mode){
        _mode = mode;
    }

    void Navigator::setNavSubMode(ENavSubMode sub_mode){
        _sub_mode = sub_mode;
    }

    void Navigator::startNavigation(){
        while(_lock_guard) hal::waitusec(1);
        _lock_guard = true;
        while(!_nav_cmd_queue.empty()) _nav_cmd_queue.pop_front();
        _nav_cmd_queue.push_back(ENavCommand::DO_FIRST_MOVE);
        _lock_guard = false;
        
        _done_outward = false;
        _updateDestination();
        _maze.updateStartSectionWall();
        _maze.makeSearchMap(_x_dest, _y_dest);
        _navigating = true;
        _is_failsafe = false;
    }

    void Navigator::endNavigation(){
        if(_lock_guard) hal::waitusec(1);
        _lock_guard = true;
        while(!_nav_cmd_queue.empty()) _nav_cmd_queue.pop_front();
        _navigating = false;
        setNavMode(ENavMode::STANDBY);
        setNavSubMode(ENavSubMode::STANDBY);
        _lock_guard = false;
    }
    
    void Navigator::update1(){
        if(_lock_guard) return;

        // パラメータの更新
        _updateParam();
        
        // 壁制御を禁止するかの判定
        _updateWallEnable();
        
        // フェールセーフ判定
        _is_failsafe = _isFailsafe();
        if(_is_failsafe){
            _navigating = false;            
        }


        
        // 現在区画の更新
        _x_cur = (uint8_t)(_x / 0.09f);
        _y_cur = (uint8_t)(_y / 0.09f);
        _azimuth = yaw2Azimuth(_yaw_setp);
                    
        // 壁の更新エリアに入ったときの動作
        bool pre_in_read_wall_area = _in_read_wall_area;
        _in_read_wall_area = _inReadWallArea(_read_wall_offset1, _read_wall_offset2);
        if(_in_read_wall_area && !pre_in_read_wall_area && _navigating){                
            // 迷路の壁情報更新
            if(!_maze.isReached(_x_cur, _y_cur)){
                _maze.updateWall(_x_cur, _y_cur, _azimuth, _ws_msg);
            }

            if(_x_cur == _x_dest && _y_cur == _y_dest && _sub_mode == ENavSubMode::START2GOAL){                
                _nav_cmd_queue.push_back(ENavCommand::GO_CENTER);                                
                _navigating = false;
            }
            else if(_x_cur == _x_dest && _y_cur == _y_dest && _sub_mode == ENavSubMode::START2GOAL2START && _done_outward){                
                _nav_cmd_queue.push_back(ENavCommand::GO_CENTER);                
                _navigating = false;
            }
            else{
                _lock_guard = true;
                _nav_cmd_queue.push_back(ENavCommand::UPDATE_POTENTIAL_MAP);                
                _nav_cmd_queue.push_back(ENavCommand::GO_NEXT_SECTION);
                _lock_guard = false;

                if(_x_cur == _x_goal && _y_cur == _y_goal){
                    _done_outward;
                }
            }
        }
        // 目標区画の更新
        _updateDestination();
        
        // navStateMsgをpublish
        _publish();
    }

    void Navigator::updateInMainLoop(){        
        _lock_guard = true;
        if(!_nav_cmd_queue.empty()){
            ENavCommand cmd = _nav_cmd_queue.front();
            _nav_cmd_queue.pop_front();
            _lock_guard = false;
            if(cmd == ENavCommand::DO_FIRST_MOVE){         
                PRINTF_ASYNC("DO_FIRST_MOVE\n");
                float target_dist = _wall2mouse_center_dist + 0.045f + _read_wall_offset2;
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, target_dist, 0.0, _v, _v, _a, _a);
            }
            else if(cmd == ENavCommand::GO_NEXT_SECTION){
                EAzimuth dest_dir = _maze.getSearchDirection2(_x_cur, _y_cur, _azimuth);
                int8_t rot_times = _maze.calcRotTimes(dest_dir, _azimuth);
                if(rot_times == 0) {
                    StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.09, _v);
                } 
                else if ( std::abs(rot_times) == 4) {                    
                    StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.03f - _read_wall_offset2, _v, _v, 0.0f, _a, _a);
                    StopFactory::push(0.1f);
                    SpinTurnFactory::push(180.0f * DEG2RAD, _yawrate_max, _yawacc);
                    StopFactory::push(0.1f);
                    StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.03f + _read_wall_offset2, 0.0f, _v, _v, _a, _a);
                }
                else if(rot_times == 2) {
                    CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, ETurnDir::CCW, -_read_wall_offset2 , _read_wall_offset2);
                }
                else if(rot_times == -2){
                    CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, ETurnDir::CW, -_read_wall_offset2 , _read_wall_offset2);
                }                        
            }            
            else if(cmd == ENavCommand::GO_CENTER){
                float target_dist = _wall2mouse_center_dist + 0.045f + _read_wall_offset2;
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, target_dist, _v, _v, 0.0f, _a, _a);
                StopFactory::push(1.0f);
            }
            else if(cmd == ENavCommand::UPDATE_POTENTIAL_MAP){
                _maze.makeSearchMap(_x_dest, _y_dest);
            }                        
        }
        _lock_guard = false;       

    }

    Maze& Navigator::getMazeRef(){
        return _maze;
    }

    void Navigator::testPmap(){
        PRINTF_ASYNC("  -- test update potential map --\n");
        uint64_t start_time = hal::getElapsedUsec();
        _maze.makeSearchMap(29, 29);
        uint64_t elapsed_time = hal::getElapsedUsec() - start_time;
        PRINTF_ASYNC("  elapsed time : %d [us]\n", elapsed_time);
    }

    void Navigator::_updateParam(){
        ParameterManager& pm = ParameterManager::getInstance();
        _x_goal = pm.goal_x;
        _y_goal = pm.goal_y;
        _v = pm.v_search_run;
        _a = pm.a_search_run;
        _yawrate_max = pm.spin_yawrate_max * DEG2RAD;
        _yawacc = pm.spin_yawacc * DEG2RAD;
        _wall2mouse_center_dist = pm.wall2mouse_center_dist;
        
        copyMsg(msg_id::WALL_SENSOR, &_ws_msg);

        VehiclePositionMsg pos_msg;
        VehicleAttitudeMsg att_msg;
        CtrlSetpointMsg setp_msg;
        copyMsg(msg_id::VEHICLE_POSITION, &pos_msg);
        copyMsg(msg_id::VEHICLE_ATTITUDE, &att_msg);
        copyMsg(msg_id::CTRL_SETPOINT, & setp_msg);

        _x = pos_msg.x;
        _y = pos_msg.y;
        _yaw = att_msg.yaw;

        _x_setp = setp_msg.x;
        _y_setp = setp_msg.y;
        _yaw_setp = setp_msg.yaw;
    }

    void Navigator::_updateWallEnable(){
        // 迷路の壁が現在の位置から見えるかの判定
        _r_wall_enable = _existLWall(_x, _y, _azimuth);
        _l_wall_enable = _existRWall(_x, _y, _azimuth);            
        // 前壁が近すぎる場合は壁制御を禁止
        if(_ws_msg.dist_a < 0.08f){
            _r_wall_enable = false;
            _l_wall_enable = false;
        }
        // 柱を見ている際は柱に近づき過ぎている場合のみ壁制御をかける
        if(_watchedPillar(_x, _y, _azimuth)){
            if(_ws_msg.dist_r > 0.045f) _r_wall_enable = false;
            if(_ws_msg.dist_l > 0.045f) _l_wall_enable = false;
        }
    }

    void Navigator::_updateDestination(){
        if(!_navigating) return;
        
        if(_sub_mode == ENavSubMode::START2GOAL){
            _x_dest = _x_goal;
            _y_dest = _y_goal;
        }            
        else if(_sub_mode == ENavSubMode::START2GOAL2START){
            if(!_done_outward){
                _x_dest = _x_goal;
                _y_dest = _y_goal;
            }
            else{
                _x_dest = 0;
                _y_dest = 0;
            }
        }
        else {
            _x_dest = 0;
            _y_dest = 0;
        }
    }

    bool Navigator::_isFailsafe(){
        if(std::fabs(_x - _x_setp) > 0.02f || std::fabs(_y - _y_setp) > 0.02f){
            return true;
        }
        return false;
    }

    bool Navigator::_inReadWallArea(float offset1, float offset2){
        float fmod_x = fmodf(_x_setp, 0.09f);
        float fmod_y = fmodf(_y_setp, 0.09f);

        if(_azimuth == EAzimuth::E) return (fmod_x >= offset1 && fmod_x <= offset2);    
        else if(_azimuth == EAzimuth::N) return (fmod_y >= offset1 && fmod_y <= offset2);
        else if(_azimuth == EAzimuth::W) return (fmod_x >= 0.09f - offset2 && fmod_x <= 0.09f - offset1);            
        else if(_azimuth == EAzimuth::S) return (fmod_y >= 0.09f - offset2 && fmod_y <= 0.09f - offset1);
        else return false;        
    }

    bool Navigator::_existRWall(float x, float y, EAzimuth azimuth) {
        // マウスと同じ角度の座標系における区画の入口からの距離
        float fmod_x = fmodf(x, 0.09f);
        float fmod_y = fmodf(y, 0.09f);
        float dist = 0.0f;
        if(azimuth == EAzimuth::E) {
            dist = fmod_x;
        }
        if(azimuth == EAzimuth::N) {
            dist = fmod_y;
        }
        if(azimuth == EAzimuth::W) {
            dist = 0.09f - fmod_x;
        }
        if(azimuth == EAzimuth::S) {
            dist = 0.09f - fmod_y;
        }

        uint8_t x_int = floorf(x);
        uint8_t y_int = floorf(y);
        if(dist > 0.045f) {
            if (azimuth == EAzimuth::E) x_int++;
            else if (azimuth == EAzimuth::N) y_int++;
            else if (azimuth == EAzimuth::W) x_int--;
            else if (azimuth == EAzimuth::S) y_int--;
        }

        if(!_maze.watchedRWall(x_int, y_int, azimuth)){
            return true;
        }
        else{
            return _maze.existRWall(x_int, y_int, azimuth);
        }
    }

    bool Navigator::_existLWall(float x, float y, EAzimuth azimuth) {
        // マウスと同じ角度の座標系における区画の入口からの距離
        float fmod_x = fmodf(x, 0.09f);
        float fmod_y = fmodf(y, 0.09f);
        float dist = 0.0f;
        if(azimuth == EAzimuth::E) {
            dist = fmod_x;
        }
        if(azimuth == EAzimuth::N) {
            dist = fmod_y;
        }
        if(azimuth == EAzimuth::W) {
            dist = 0.09f - fmod_x;
        }
        if(azimuth == EAzimuth::S) {
            dist = 0.09f - fmod_y;
        }

        uint8_t x_int = floorf(x);
        uint8_t y_int = floorf(y);
        if(dist > 0.045f) {
            if (azimuth == EAzimuth::E) x_int++;
            else if (azimuth == EAzimuth::N) y_int++;
            else if (azimuth == EAzimuth::W) x_int--;
            else if (azimuth == EAzimuth::S) y_int--;
        }

        if(!_maze.watchedLWall(x_int, y_int, azimuth)){
            return true;
        }
        else{
            return _maze.existLWall(x_int, y_int, azimuth);
        }
    }

    bool Navigator::_watchedPillar(float x, float y, EAzimuth azimuth){
        // マウスと同じ角度の座標系における区画の入口からの距離
        float fmod_x = fmodf(x, 0.09f);
        float fmod_y = fmodf(y, 0.09f);
        float dist = 0.0f;
        if(azimuth == EAzimuth::E) {
            dist = fmod_x;
        }
        if(azimuth == EAzimuth::N) {
            dist = fmod_y;
        }
        if(azimuth == EAzimuth::W) {
            dist = 0.09f - fmod_x;
        }
        if(azimuth == EAzimuth::S) {
            dist = 0.09f - fmod_y;
        }
        
        if(dist > 0.045f && dist < 0.06f){
            return true;
        }
        else{
            return false;
        }
    }

    void Navigator::_publish(){
        NavStateMsg msg;            
        msg.navigating = _navigating;
        msg.mode = _mode;
        msg.sub_mode = _sub_mode;
        msg.x_cur = _x_cur;
        msg.y_cur = _y_cur;
        msg.azimuth = _azimuth;
        msg.is_failsafe = _is_failsafe;
        publishMsg(msg_id::NAV_STATE, &msg);
    }

    int usrcmd_navigator(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "test_pmap") == 0) {
            Navigator::getInstance().testPmap();
            return 0;
        }

    
        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    }

}

