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

// Module
#include "parameterManager.h"


namespace module{

    Navigator::Navigator(){
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
        _nav_cmd_queue.push(ENavCommand::DO_FIRST_MOVE);
        _lock_guard = false;
        
        _done_outward = false;
        _updateDestination();
        maze.makeSearchMap(_x_dest, _y_dest);
        _navigating = true;        
    }

    void Navigator::endNavigation(){
        if(_lock_guard) hal::waitusec(1);
        _lock_guard = true;
        while(!_nav_cmd_queue.empty()) _nav_cmd_queue.pop_front();
        _navigating = false;
        _lock_guard = false;
    }
    
    void Navigator::update1(){
        // パラメータの更新
        _updateParam();
        
        // 壁制御を禁止するかの判定
        _updateWallEnable();
        
        // フェールセーフ判定
        _is_failsafe = _isFailsafe();
        
        // 現在区画の更新
        _x_cur = (uint8_t)(_x / 0.09f);
        _y_cur = (uint8_t)(_y / 0.09f);
        _azimuth = yaw2Azimuth(_yaw);
                    
        // 壁の更新エリアに入ったときの動作
        bool pre_in_read_wall_area = _in_read_wall_area;
        _in_read_wall_area = _inReadWallArea();
        if(_in_read_wall_area && !pre_in_read_wall_area && _navigating){                
            // 迷路の壁情報更新
            if(!_maze.isReached(_x_cur, _y_cur)){
                _maze.updateWall(_x_cur, _y_cur, _azimuth, _ws_msg);
            }

            if(_x_cur == _x_dest && _y_cur == _y_dest && _sub_mode == START2GOAL){
                _lock_guard = true;
                _nav_cmd_queue.push(ENavCommand::GO_CENTER);
                _lock_guard = false;
                
                _navigating = false;
            }
            else if(_x_cur == _x_dest && _y_cur == _y_dest && _sub_mode == START2GOAL2START && _done_outward){
                _lock_guard = true;
                _nav_cmd_queue.push(ENavCommand::GO_CENTER);
                _lock_guard = false;

                _navigating = false;
            }
            else{
                _lock_guard = true;
                _nav_cmd_queue.push(ENavCommand::UPDATE_POTENTIAL_MAP);
                _nav_cmd_queue.push(ENavCommand::CALC_NEXT_SECTION);
                _nav_cmd_queue.push(ENavCommand::GO_NEXT_SECTION);
                _lock_guard = false;

                if(_x_cur == _x_goal && _y_cur == y_goal){
                    _done_outward;
                }
            }
        }
        // 目標区画の更新
        _updateDestination();
        
        // navStateMsgをpublish
        _publish();
    }

    Maze& getMazeRef(){
        return
    }

    void updateSearchMap(){
        maze.makeSearchMap(_x_dest, _y_dest);
    }

    void _updateWallEnable(){
        // 迷路の壁が現在の位置から見えるかの判定
        _r_wall_enable = _existLWall(_x, _y, _azimuth);
        _l_wall_enable = _existRWall(_x, _y, _azimuth);            
        // 前壁が近すぎる場合は壁制御を禁止
        if(_dist_a < 0.08f){
            _r_wall_enable = false;
            _l_wall_enable = false;
        }
        // 柱を見ている際は柱に近づき過ぎている場合のみ壁制御をかける
        if(_watchedPillar(_x, _y, _azimuth)){
            if(_dist_r > 0.045f) _r_wall_enable = false;
            if(_dist_l > 0.045f) _l_wall_enable = false;
        }
    }

    bool Navigator::_updateDestination(){
        if(!_navigating) return;
        
        if(_sub_mode ==ENavMode::START2GOAL){
            _x_dest = _x_goal;
            _y_dest = _y_goal;
        }            
        else if(_sub_mode == ENavMode::START2GOAL2START){
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
        return false;
    }

    void Navigator::_updateParam(){
        ParameterManager& pm = ParameterManager::getInstance();
        _x_goal = pm.goal_x;
        _y_goal = pm.goal_y;        
        
        copyMsg(msg_id::WALL_SENSOR, &_ws_msg);

        VehiclePositionMsg pos_msg;
        copyMsg(msg_id::VEHICLE_POSITION, &pos_msg);
        _x = pos_msg.x;
        _y = pos_msg.y;
        _yaw = pos_msg.yaw;
    }

    void Navigator::_publish(){
        NavStateMsg msg;            
        msg.navigating = _navigating;
        msg.mode = _mode;
        msg.sub_mode = _sub_mode;
        msg.x_cur = _x_cur;
        msg.y_cur = _y_cur;
        msg.x_next = _x_next;
        msg.y_next = _y_next;
        msg.azimuth = _azimuth;
        msg.is_failsafe = _is_failsafe;
        publishMsg(msg_id::NAV_STATE, &msg);
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

    bool Navigator:: _inReadWallArea(float offset1, float offset2){
        float fmod_x = fmodf(_x, 0.09f);
        float fmod_y = fmodf(_y, 0.09f);

        if(_azimuth == EAzimuth::E) return (fmod_x >= offset1 && fmod_x <= offset2);    
        else if(_azimuth == EAzimuth::N) return (fmod_y >= offset1 && fmod_y <= offset2);
        else if(_azimuth == EAzimuth::W) return (fmod_x >= 0.09f - offset2 && fmod_x <= 0.09f - offset1);            
        else if(_azimuth == EAzimuth::S) return (fmod_y >= 0.09f - offset2 && fmod_y <= 0.09f - offset1);
        else return false;        
    }

    void Navigator::testPmap(){
        PRINTF_ASYNC("  -- test update potential map --\n");
        uint64_t start_time = hal::getElapsedUsec();
        _maze.makeSearchMap(29, 29);
        uint64_t elapsed_time = hal::getElapsedUsec() - start_time;
        PRINTF_ASYNC("  elapsed time : %d [us]\n", elapsed_time);
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

