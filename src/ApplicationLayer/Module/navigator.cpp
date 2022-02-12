#include "navigator.h"

#include <cmath>
#include <string>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_timer.h"
#include "hal_critical_section.h"

// Msg
#include "msgBroker.h"
#include "vehiclePositionMsg.h"
#include "vehicleAttitudeMsg.h"

// Module
#include "parameterManager.h"
#include "trajectoryFactory.h"
#include "ledController.h"


namespace module{

    Navigator::Navigator() :
        _mode(ENavMode::MODE_SELECT),
        _sub_mode(ENavSubMode::STANDBY),
        _ctrl_mode(ECtrlMode::DIRECT_DUTY_SET),
        _lock_guard(false),
        _navigating(false),
        _done_outward(false),
        _x_cur(0),
        _y_cur(0),
        _x_last(255),
        _y_last(255),
        _azimuth(EAzimuth::N),
        _x_dest(7),
        _y_dest(7),
        _x_goal(7),
        _y_goal(7),
        _r_wall_enable(false),
        _l_wall_enable(false),
        _is_actuator_error(false),
        _is_failsafe(false),
        _in_read_wall_area(false),
        _x(0.045f),
        _y(0.045f),
        _yaw(90.0f * DEG2RAD),
        _x_setp(0.045f),
        _y_setp(0.045f),
        _yaw_setp(90.0f * DEG2RAD),
        _turn_type(ETurnType::NONE),
        _v(0.25f),
        _a(4.0f),
        _yawrate_max(1000.0f * DEG2RAD),
        _yawacc(1000.0f * DEG2RAD),
        _wall2mouse_center_dist(0.0f),
        _read_wall_offset1(0.012f),
        _read_wall_offset2(0.0135f),
        _is_pre_read_l_wall(false),
        _is_pre_read_r_wall(false),
        _pre_read_l_wall_dist(0.0f),
        _pre_read_r_wall_dist(0.0f)
    {
        setModuleName("Navigator");
        _maze.readMazeDataFromFlash();
    }
    
    void Navigator::setNavMode(ENavMode mode){
        _mode = mode;
        _is_failsafe = false;
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
        _is_failsafe = false;
        _navigating = true;              
    }

    void Navigator::endNavigation(){
        if(_lock_guard) hal::waitusec(1);
        _lock_guard = true;
        while(!_nav_cmd_queue.empty()) _nav_cmd_queue.pop_front();
        
        _x_last = 255;
        _y_last = 255;
        _navigating = false;
        setNavMode(ENavMode::STANDBY);
        setNavSubMode(ENavSubMode::STANDBY);
        _lock_guard = false;
    }
    
    void Navigator::update0(){
        if(_lock_guard) return;

        // パラメータの更新
        _updateParam();
        
        // 壁制御を禁止するかの判定
        _updateWallEnable();
        
        // フェールセーフ判定
        bool is_failsafe_pre = _is_failsafe;
        _is_failsafe = _isFailsafe();
        
        if(_is_failsafe){
            _navigating = false;
        }

        if(!is_failsafe_pre && _is_failsafe){
            PRINTF_PICKLE("!!!!FAILSAFE!!!!      | cor_setp:(%6.3f, %6.3f)| x_setp:%6.3f, x:%6.3f |y_setp %6.3f, y:%6.3f |aout_err:%d\n",_x_setp/0.09f,_y_setp/0.09f, _x_setp, _x, _y_setp, _y, _is_actuator_error);
        }

        // 現在区画の更新
        _x_cur = (uint8_t)(_x / 0.09f);
        _y_cur = (uint8_t)(_y / 0.09f);
        _azimuth = yaw2Azimuth(_yaw);

        // 横壁の読み取り
        if(_inReadWallArea(0.078f, 0.08f) && _mode == ENavMode::SEARCH ){
            _is_pre_read_l_wall = _ws_msg.is_left;
            _is_pre_read_r_wall = _ws_msg.is_right;
            _pre_read_l_wall_dist = _ws_msg.dist_l;
            _pre_read_r_wall_dist = _ws_msg.dist_r;
            module::LedController::getInstance().oneshotFcled(0, 1, 0, 0.005, 0.005);
        }

        // 壁の更新エリアに入ったときの動作
        bool pre_in_read_wall_area = _in_read_wall_area;
        _in_read_wall_area = _inReadWallArea(_read_wall_offset1, _read_wall_offset2);
        if(_in_read_wall_area && !pre_in_read_wall_area && 
           !(_x_last == _x_cur && _y_last == _y_cur ) &&
           _navigating &&
           _mode == ENavMode::SEARCH
        ){                            
            // LEDの点灯
            module::LedController::getInstance().oneshotFcled(1, 1, 0, 0.005, 0.005);

            // 迷路の壁情報更新
            if(!_maze.isReached(_x_cur, _y_cur)){                               
                //PRINTF_PICKLE("UPDATE_WALL          | x_setp:%6.3f, y_setp:%6.3f | x:%6.3f, y:%6.3f | azimuth:%c\n",_x_setp/0.09f, _y_setp/0.09f, _x/0.09f, _y/0.09f, azimuth2Char(_azimuth));
                WallSensorMsg ws_msg_temp = _ws_msg;
                ws_msg_temp.dist_l = _pre_read_l_wall_dist;
                ws_msg_temp.dist_r = _pre_read_r_wall_dist;
                ws_msg_temp.is_left = _is_pre_read_l_wall;
                ws_msg_temp.is_right = _is_pre_read_r_wall;                
                //PRINTF_PICKLE("  dist: %.3f, %.3f, %.3f, %.3f\n", _ws_msg.dist_al, _ws_msg.dist_l, _ws_msg.dist_r, _ws_msg.dist_ar);
                //PRINTF_PICKLE("  dist: %.3f, %.3f, %.3f, %.3f\n", ws_msg_temp.dist_al, ws_msg_temp.dist_l, ws_msg_temp.dist_r, ws_msg_temp.dist_ar);
                _maze.updateWall(_x_cur, _y_cur, _azimuth, ws_msg_temp);
            }

            if(_x_cur == _x_dest && _y_cur == _y_dest && 
               (_sub_mode == ENavSubMode::START2GOAL ||
                (_sub_mode == ENavSubMode::START2GOAL2START && _done_outward)
               )
            ){                
                _nav_cmd_queue.push_back(ENavCommand::GO_CENTER);
                _nav_cmd_queue.push_back(ENavCommand::SAVE_MAZE);
                _navigating = false;
            }
            else{
                if(_x_cur == _x_goal && _y_cur == _y_goal){
                    _done_outward = true;
                    module::LedController::getInstance().flashFcled(0, 1, 0, 0.5, 0.5);
                    _nav_cmd_queue.push_back(ENavCommand::SAVE_MAZE);
                }
                // 目標区画の更新
                _updateDestination();

                _x_last = _x_cur;
                _y_last = _y_cur;
                _lock_guard = true;
                _nav_cmd_queue.push_back(ENavCommand::UPDATE_POTENTIAL_MAP);                
                _nav_cmd_queue.push_back(ENavCommand::GO_NEXT_SECTION);
                _lock_guard = false;

            }
        }
        
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
                float target_dist = _wall2mouse_center_dist + 0.045f + _read_wall_offset2;
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, target_dist, 0.0, _v, _v, _a, _a);
                PRINTF_PICKLE("DO_FIRST_MOVE        | x_setp:%6.3f, y_setp:%6.3f | x:%6.3f, y:%6.3f\n",_x_setp/0.09f, _y_setp/0.09f, _x/0.09f, _y/0.09f);
            }
            else if(cmd == ENavCommand::GO_NEXT_SECTION){
                EAzimuth dest_dir = _maze.getSearchDirection(_x_cur, _y_cur, _azimuth);
                EAzimuth min_dir = _maze.getMinDirection(_x_cur, _y_cur, _azimuth);
                EAzimuth unknown_dir = _maze.getUnknownDirection(_x_cur, _y_cur, _azimuth);
                int8_t rot_times = _maze.calcRotTimes(dest_dir, _azimuth);
                if(rot_times == 0) {
                    StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.09, _v);
                } 
                else if (std::abs(rot_times) == 4) {
                    //_nav_cmd_queue.push_back(ENavCommand::SAVE_MAZE);   
                    if(_maze.existsAWall(_x_cur, _y_cur, _azimuth)){
                        StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f - _read_wall_offset2 - 0.02f, _v, _v, _v, _a, _a);                        
                        AheadWallCorrectionFactory::push(0.2f, 0.05f);
                        if(_maze.existsRWall(_x_cur, _y_cur, _azimuth)){
                            SpinTurnFactory::push(-90.0f * DEG2RAD, _yawrate_max, _yawacc);                            
                            AheadWallCorrectionFactory::push(0.2f, 0.1f, true);
                            SpinTurnFactory::push(-90.0f * DEG2RAD, _yawrate_max, _yawacc);                            
                        }
                        else if(_maze.existsLWall(_x_cur, _y_cur, _azimuth)){
                            SpinTurnFactory::push(90.0f * DEG2RAD, _yawrate_max, _yawacc);                            
                            AheadWallCorrectionFactory::push(0.2f, 0.1f, true);
                            SpinTurnFactory::push(90.0f * DEG2RAD, _yawrate_max, _yawacc);                            
                        }
                        else{
                            SpinTurnFactory::push(180.0f * DEG2RAD, _yawrate_max, _yawacc);                           
                        }
                    }else{
                        StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f - _read_wall_offset2, _v, _v, 0.0f, _a, _a);                       
                        SpinTurnFactory::push(180.0f * DEG2RAD, _yawrate_max, _yawacc);                        
                    }
                    StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045f + _read_wall_offset2, 0.0f, _v, _v, _a, _a);
                }
                else if(rot_times == 2) {
                    CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, ETurnDir::CCW, -_read_wall_offset2 , _read_wall_offset2);
                }
                else if(rot_times == -2){
                    CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, ETurnDir::CW, -_read_wall_offset2 , _read_wall_offset2);
                }
                PRINTF_PICKLE("GO_NEXT_SECTION      | x_setp:%6.3f, y_setp:%6.3f | x:%6.3f, y:%6.3f\n",_x_setp/0.09f, _y_setp/0.09f, _x/0.09f, _y/0.09f);
                PRINTF_PICKLE("  dest_dir:%c, azimuth:%c, rot_times:%d\n",azimuth2Char(dest_dir), azimuth2Char(_azimuth), rot_times);
                PRINTF_PICKLE("  min_dir :%c, unknown:%c\n", azimuth2Char(min_dir), azimuth2Char(unknown_dir));
                _printWall();
            }            
            else if(cmd == ENavCommand::GO_CENTER){
                float target_dist = 0.045f - _read_wall_offset2;
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, target_dist, _v, _v, 0.0f, _a, _a);
                AheadWallCorrectionFactory::push(0.2f, 0.05f);
                StopFactory::push(1.0f);
            }
            else if(cmd == ENavCommand::UPDATE_POTENTIAL_MAP){
                _maze.makeSearchMap(_x_dest, _y_dest);
                PRINTF_PICKLE("UPDATE_POTENTIAL_MAP | x_setp:%6.3f, y_setp:%6.3f | x:%6.3f, y:%6.3f\n",_x_setp/0.09f, _y_setp/0.09f, _x/0.09f, _y/0.09f);
            }
            else if(cmd == ENavCommand::SAVE_MAZE){
                //hal::enterCriticalSection();
                _maze.writeMazeData2Flash();
                //hal::leaveCriticalSection();
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

    void Navigator::printMaze(){
        
        for(uint8_t i=0;i<32;i++){
            PRINTF_ASYNC("+");
            for(uint8_t j=0;j<32;j++){
                std::string h_wall;
                if(_maze.readWall(j,31-i).N){
                	h_wall = "---";
                }
                else{
                	h_wall = "   ";
                }

                if(j==31){
                	PRINTF_ASYNC("%s+\n", h_wall.c_str());
                }
                else{
                	PRINTF_ASYNC("%s+", h_wall.c_str());
                }
            }
            for(uint8_t j=0;j<32;j++){
                std::string v_wall;
                if(_maze.readWall(j,31-i).W){
                	if(_maze.isReached(j,31-i)){
                		v_wall = "|   ";
                	}
                	else{
                		v_wall = "| N ";
                	}
                }
                else{
                	if(_maze.isReached(j,31-i)){
                		v_wall = "    ";
                	}
                	else{
                		v_wall = "  N ";
                	}
                }
                PRINTF_ASYNC("%s", v_wall.c_str());                
            }
            PRINTF_ASYNC("|\n");
        }
        PRINTF_ASYNC("+");
        for(uint8_t j=0;j<32;j++){
            std::string h_wall;
            if(_maze.readWall(j,0).S) h_wall = "---";
            else h_wall = "   ";                
            if(j==31) h_wall += "+\n";
            PRINTF_ASYNC("%s+", h_wall.c_str());
        }

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
        ActuatorOutputMsg aout_msg;
        copyMsg(msg_id::VEHICLE_POSITION, &pos_msg);
        copyMsg(msg_id::VEHICLE_ATTITUDE, &att_msg);
        copyMsg(msg_id::CTRL_SETPOINT, &setp_msg);
        copyMsg(msg_id::ACTUATOR_OUTPUT, &aout_msg);

        _x = pos_msg.x;
        _y = pos_msg.y;
        _yaw = att_msg.yaw;

        _x_setp = setp_msg.x;
        _y_setp = setp_msg.y;
        _yaw_setp = setp_msg.yaw;
        _turn_type = setp_msg.turn_type;

        _is_actuator_error = aout_msg.is_error;
        _ctrl_mode = aout_msg.ctrl_mode;
    }

    void Navigator::_updateWallEnable(){
        // 迷路の壁が現在の位置から見えるかの判定
        if(_mode == ENavMode::SEARCH || _mode == ENavMode::FASTEST){
            _r_wall_enable = _existsRWall(_x, _y, _azimuth) && _ws_msg.is_right_ctrl;
            _l_wall_enable = _existsLWall(_x, _y, _azimuth) && _ws_msg.is_left_ctrl;
        }
        else{
            _r_wall_enable = _ws_msg.is_right_ctrl;
            _l_wall_enable = _ws_msg.is_left_ctrl;
        }
        
        // 前壁が近すぎる場合は壁制御を禁止
        if(_ws_msg.dist_al < 0.08f && _ws_msg.dist_ar < 0.08f){
            _r_wall_enable = false;
            _l_wall_enable = false;
        }
        
        // 柱を見ている際は柱に近づき過ぎている場合のみ壁制御をかける
        if(_watchedPillar(_x, _y, _azimuth) && _mode == ENavMode::SEARCH){
            if(_ws_msg.dist_r > 0.035f) _r_wall_enable = false;
            if(_ws_msg.dist_l > 0.035f) _l_wall_enable = false;
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
        float x_thr = 0.045f;
        float y_thr = 0.045f;
        if(_mode == ENavMode::SEARCH){
            x_thr = 0.1f;
            y_thr = 0.1f;
        }
        else{
            x_thr = 0.1f;
            y_thr = 0.1f;
        }
        
        return (_turn_type != ETurnType::AHEAD_WALL_CORRECTION &&
                _turn_type != ETurnType::AHEAD_WALL_YAW_CORRECTION &&
            _ctrl_mode == ECtrlMode::VEHICLE &&
           (std::fabs(_x - _x_setp) > x_thr || 
            std::fabs(_y - _y_setp) > y_thr //||
            //std::fabs(_yaw - _yaw_setp) > 20.0f * DEG2RAD
            //||
            //_is_actuator_error
           )
        );
    }

    bool Navigator::_inReadWallArea(float offset_pre, float offset_fol){
        float fmod_x = fmodf(_x, 0.09f);
        float fmod_y = fmodf(_y, 0.09f);

        if(_azimuth == EAzimuth::E) return (fmod_x >= offset_pre && fmod_x <= offset_fol);
        else if(_azimuth == EAzimuth::N) return (fmod_y >= offset_pre && fmod_y <= offset_fol);
        else if(_azimuth == EAzimuth::W) return (fmod_x >= 0.09f - offset_fol && fmod_x <= 0.09f - offset_pre);
        else if(_azimuth == EAzimuth::S) return (fmod_y >= 0.09f - offset_fol && fmod_y <= 0.09f - offset_pre);
        else return false;        
    }

    bool Navigator::_existsRWall(float x, float y, EAzimuth azimuth) {
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

        uint8_t x_int = (uint8_t)(x / 0.09f);
        uint8_t y_int = (uint8_t)(y / 0.09f);
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
            return _maze.existsRWall(x_int, y_int, azimuth);
        }
    }

    bool Navigator::_existsLWall(float x, float y, EAzimuth azimuth) {
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

        uint8_t x_int = (uint8_t)(x / 0.09f);
        uint8_t y_int = (uint8_t)(y / 0.09f);
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
            return _maze.existsLWall(x_int, y_int, azimuth);
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
        
        if(dist > 0.05f && dist < 0.065f){
            return true;
        }
        else{
            return false;
        }
    }

    void Navigator::_printWall(){
        Wall wall = _maze.readWall(_x_cur, _y_cur);
        char wall_e = (wall.E == 1 ? '|' : ' ');
        char wall_n = (wall.N == 1 ? '-' : ' ');
        char wall_w = (wall.W == 1 ? '|' : ' ');
        char wall_s = (wall.S == 1 ? '-' : ' ');
        uint16_t p_e = 0xffff;
        uint16_t p_n = 0xffff;
        uint16_t p_w = 0xffff;
        uint16_t p_s = 0xffff;
        if(wall.E == 0 && wall.EF == 1 && _x_cur != 31) p_e = _maze.p_map[_x_cur+1][_y_cur];
        if(wall.N == 0 && wall.NF == 1 && _y_cur != 31) p_n = _maze.p_map[_x_cur][_y_cur+1];
        if(wall.W == 0 && wall.WF == 1 && _x_cur != 0) p_w = _maze.p_map[_x_cur-1][_y_cur];
        if(wall.S == 0 && wall.SF == 1 && _y_cur != 0) p_s = _maze.p_map[_x_cur][_y_cur-1];
        char r_e = (_maze.isReached(_x_cur+1, _y_cur)   == 1 ? 'R' : 'N');
        char r_n = (_maze.isReached(_x_cur  , _y_cur+1) == 1 ? 'R' : 'N');
        char r_w = (_maze.isReached(_x_cur-1, _y_cur)   == 1 ? 'R' : 'N');
        char r_s = (_maze.isReached(_x_cur  , _y_cur-1) == 1 ? 'R' : 'N');

        PRINTF_PICKLE("\n");
        PRINTF_PICKLE("   (%2d, %2d)\n", _x_cur, _y_cur);
        PRINTF_PICKLE("         %c%-5d \n", r_n, p_n);
        PRINTF_PICKLE("          + %c +\n", wall_n);
        PRINTF_PICKLE("   %c%-5d %c %c %c %c%-5d\n", r_w, p_w, wall_w, azimuth2Char(_azimuth), wall_e, r_e, p_e);
        PRINTF_PICKLE("          + %c +  \n", wall_s);
        PRINTF_PICKLE("         %c%-5d  \n\n", r_s, p_s);
    }

    void Navigator::debugWall(uint8_t x, uint8_t y){
        Wall wall = _maze.readWall(x, y);
        char wall_e = (wall.E == 1 ? '|' : ' ');
        char wall_n = (wall.N == 1 ? '-' : ' ');
        char wall_w = (wall.W == 1 ? '|' : ' ');
        char wall_s = (wall.S == 1 ? '-' : ' ');
        uint16_t p_e = 0xffff;
        uint16_t p_n = 0xffff;
        uint16_t p_w = 0xffff;
        uint16_t p_s = 0xffff;
        if(wall.E == 0 && wall.EF == 1 && x != 31) p_e = _maze.p_map[x+1][y];
        if(wall.N == 0 && wall.NF == 1 && y != 31) p_n = _maze.p_map[x][y+1];
        if(wall.W == 0 && wall.WF == 1 && x != 0) p_w = _maze.p_map[x-1][y];
        if(wall.S == 0 && wall.SF == 1 && y != 0) p_s = _maze.p_map[x][y-1];
        char r_e = (_maze.isReached(x+1, y)   == 1 ? 'R' : 'N');
        char r_n = (_maze.isReached(x  , y+1) == 1 ? 'R' : 'N');
        char r_w = (_maze.isReached(x-1, y)   == 1 ? 'R' : 'N');
        char r_s = (_maze.isReached(x  , y-1) == 1 ? 'R' : 'N');

        PRINTF_ASYNC("\n");
        PRINTF_ASYNC("   (%2d, %2d)\n", x, y);
        PRINTF_ASYNC("         %c%-5d \n", r_n, p_n);
        PRINTF_ASYNC("          + %c +\n", wall_n);
        PRINTF_ASYNC("   %c%-5d %c %c %c %c%-5d\n", r_w, p_w, wall_w, azimuth2Char(_azimuth), wall_e, r_e, p_e);
        PRINTF_ASYNC("          + %c +  \n", wall_s);
        PRINTF_ASYNC("         %c%-5d  \n\n", r_s, p_s);
        
        PRINTF_ASYNC("  ---------\n");
        PRINTF_ASYNC("  maze.existsLWall(%d, %d, E) = %d\n", x, y, _maze.existsLWall(x, y, EAzimuth::E));
        PRINTF_ASYNC("  maze.existsRWall(%d, %d, E)  = %d\n", x, y, _maze.existsRWall(x, y, EAzimuth::E));
        PRINTF_ASYNC("  maze.watchedLWall(%d, %d, E) = %d\n", x, y, _maze.watchedLWall(x, y, EAzimuth::E));
        PRINTF_ASYNC("  maze.watchedRWall(%d, %d, E) = %d\n", x, y, _maze.watchedRWall(x, y, EAzimuth::E));
        PRINTF_ASYNC("  ---------\n");       
        PRINTF_ASYNC("  maze.existsLWall(%d, %d, N) = %d\n", x, y, _maze.existsLWall(x, y, EAzimuth::N));
        PRINTF_ASYNC("  maze.existsRWall(%d, %d, N) = %d\n", x, y, _maze.existsRWall(x, y, EAzimuth::N));
        PRINTF_ASYNC("  maze.watchedLWall(%d, %d, N) = %d\n", x, y, _maze.watchedLWall(x, y, EAzimuth::N));
        PRINTF_ASYNC("  maze.watchedRWall(%d, %d, N) = %d\n", x, y, _maze.watchedRWall(x, y, EAzimuth::N));
        PRINTF_ASYNC("  ---------\n");
        PRINTF_ASYNC("  maze.existsLWall(%d, %d, W) = %d\n", x, y, _maze.existsLWall(x, y, EAzimuth::W));
        PRINTF_ASYNC("  maze.existsRWall(%d, %d, W) = %d\n", x, y, _maze.existsRWall(x, y, EAzimuth::W));
        PRINTF_ASYNC("  maze.watchedLWall(%d, %d, W) = %d\n", x, y, _maze.watchedLWall(x, y, EAzimuth::W));
        PRINTF_ASYNC("  maze.watchedRWall(%d, %d, W) = %d\n", x, y, _maze.watchedRWall(x, y, EAzimuth::W));
        PRINTF_ASYNC("  ---------\n");
        PRINTF_ASYNC("  maze.existsLWall(%d, %d, S) = %d\n", x, y, _maze.existsLWall(x, y, EAzimuth::S));
        PRINTF_ASYNC("  maze.existsRWall(%d, %d, S) = %d\n", x, y, _maze.existsRWall(x, y, EAzimuth::S));
        PRINTF_ASYNC("  maze.watchedLWall(%d, %d, S) = %d\n", x, y, _maze.watchedLWall(x, y, EAzimuth::S));
        PRINTF_ASYNC("  maze.watchedRWall(%d, %d, S) = %d\n", x, y, _maze.watchedRWall(x, y, EAzimuth::S));
    }


    void Navigator::_publish(){
        NavStateMsg msg;            
        msg.navigating = _navigating;
        msg.mode = _mode;
        msg.sub_mode = _sub_mode;
        msg.x_cur = _x_cur;
        msg.y_cur = _y_cur;
        msg.azimuth = _azimuth;
        msg.r_wall_enable = _r_wall_enable;
        msg.l_wall_enable = _l_wall_enable;
        msg.in_read_wall_area = _in_read_wall_area;
        msg.is_failsafe = _is_failsafe;        
        publishMsg(msg_id::NAV_STATE, &msg);
    }

    int usrcmd_navigator(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  test_pmap :\r\n");
            PRINTF_ASYNC("  printMaze :\r\n");
            PRINTF_ASYNC("  debugWall <x_cor> <y_cor> :\r\n");
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "test_pmap") == 0) {
            Navigator::getInstance().testPmap();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "printMaze") == 0) {
            Navigator::getInstance().printMaze();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "debugWall") == 0) {
            if(argc != 4){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }

            std::string x_str(argv[2]);
            uint8_t x = std::stoi(x_str);

            std::string y_str(argv[3]);
            uint8_t y = std::stoi(y_str);


            Navigator::getInstance().debugWall(x, y);
            return 0;
        }

    
        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    }

}

