#include "navigator.h"

#include <cmath>

// Msg
#include "msgBroker.h"
#include "vehiclePositionMsg.h"

namespace module{

    Navigator::Navigator(){
        setModuleName("Navigator");
    }

    void Navigator::update2(){
        _updateParam();
    	// 壁制御を禁止するかの判定
        {
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

        // フェールセーフ判定
        _is_failsafe = _isFailsafe();

        _publish();    
    }

    bool Navigator::_isFailsafe(){
    	return false;
    }

    void Navigator::_updateParam(){
    	WallSensorMsg ws_msg;
    	copyMsg(msg_id::WALL_SENSOR, &ws_msg);
    	_dist_r = ws_msg.dist_r;
		_dist_l = ws_msg.dist_l;
		_dist_a = ws_msg.dist_a;


    	VehiclePositionMsg pos_msg;
    	copyMsg(msg_id::VEHICLE_POSITION, &pos_msg);
    	_x = pos_msg.x;
    	_y = pos_msg.y;
    }


    void Navigator::_publish(){
        NavStateMsg msg;
        msg.armed = _armed;
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

    void Navigator::setNavMode(ENavMode mode){
        _mode = mode;
    }

    void Navigator::setNavSubMode(ENavSubMode sub_mode){
        _sub_mode = sub_mode;
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





    int usrcmd_navigator(int argc, char **argv){
        return 0;
    }

}

