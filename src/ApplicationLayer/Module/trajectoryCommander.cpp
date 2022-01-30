#include "trajectoryCommander.h"

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"

namespace module {

    TrajectoryCommander::TrajectoryCommander() :
    _x(0.045f),
    _y(0.045f),
    _yaw(90.0f * 3.14159265f / 180.0f),
    _lock_guard(false)    
    {
        setModuleName("TrajectoryCommander");
    }

    void TrajectoryCommander::push(std::unique_ptr<BaseTrajectory>&& traj) {
        _lock_guard = true;
        if(_traj_queue.empty()) {
            traj->setInitPos(_x, _y, _yaw);
        }
        _traj_queue.push_back(std::move(traj));
        _lock_guard = false;
    }

    void TrajectoryCommander::clear() {
        _lock_guard = true;
        while(!_traj_queue.empty()) _traj_queue.pop_front();
        _lock_guard = false;
    }

    void TrajectoryCommander::reset(float x, float y, float yaw){
        clear();
        _x = x;
        _y = y;
        _yaw = yaw;
    }

    void TrajectoryCommander::update0(){
        if(_lock_guard) return;
        
        if(!_traj_queue.empty()){

            _traj_queue.front()->update();

            if(_traj_queue.front()->isEnd()){
                _x = _traj_queue.front()->getEndX();
                _y = _traj_queue.front()->getEndY();
                _yaw = _traj_queue.front()->getEndYaw();
                _traj_queue.pop_front();
                if(!_traj_queue.empty()){
                    _traj_queue.front()->setInitPos(_x, _y, _yaw);
                }
            }
        }        
        _publish();
    }

    void TrajectoryCommander::debug(){
        constexpr float RAD2DEG = 180.0f / 3.14159265f;
        CtrlSetpointMsg msg;
        copyMsg(msg_id::CTRL_SETPOINT, &msg);        
        
        PRINTF_ASYNC("  -- trajectoryCommander Val --\n");
        PRINTF_ASYNC("  x              : %f\n", _x);
        PRINTF_ASYNC("  y              : %f\n", _y);
        PRINTF_ASYNC("  yaw            : %f\n", _yaw * RAD2DEG);        
        PRINTF_ASYNC("  traj_queue num : %d\n", _traj_queue.size());
        PRINTF_ASYNC("  -- ctrlSetpointMsg --\n");
        PRINTF_ASYNC("  x              : %f\n", msg.x);
        PRINTF_ASYNC("  v_x            : %f\n", msg.v_x);
        PRINTF_ASYNC("  a_x            : %f\n", msg.a_x);
        PRINTF_ASYNC("  y              : %f\n", msg.y);
        PRINTF_ASYNC("  v_y            : %f\n", msg.v_y);
        PRINTF_ASYNC("  a_y            : %f\n", msg.a_y);
        PRINTF_ASYNC("  v_xy_body      : %f\n", msg.v_xy_body);
        PRINTF_ASYNC("  a_xy_body      : %f\n", msg.a_xy_body);
        PRINTF_ASYNC("  yaw            : %f\n", msg.yaw);
        PRINTF_ASYNC("  yawrate        : %f\n", msg.yawrate);
        PRINTF_ASYNC("  yawacc         : %f\n", msg.yawacc);
        PRINTF_ASYNC("  beta           : %f\n", msg.beta);
        PRINTF_ASYNC("  beta_dot       : %f\n", msg.beta_dot);
        PRINTF_ASYNC("  traj_type      : %s\n", trajType2Str(msg.traj_type).c_str());
        PRINTF_ASYNC("  turn_type      : %s\n", turnType2Str(msg.turn_type).c_str());
        PRINTF_ASYNC("  turn_dir       : %s\n", turnDir2Str(msg.turn_dir).c_str());


    }

    void TrajectoryCommander::_publish(){        
        if(_traj_queue.empty()){
            CtrlSetpointMsg msg;
            msg.x = _x;
            msg.v_x = 0.0f;
            msg.a_x = 0.0f;
            msg.y = _y;
            msg.v_y = 0.0f;
            msg.a_y = 0.0f;
            msg.v_xy_body = 0.0f;
            msg.a_xy_body = 0.0f;
            msg.yaw = _yaw;
            msg.yawrate = 0.0f;
            msg.yawacc = 0.0f;
            msg.beta = 0.0f;
            msg.beta_dot = 0.0f;
            msg.traj_type = ETrajType::NONE;
            msg.turn_type = ETurnType::NONE;
            msg.turn_dir = ETurnDir::NO_TURN;
            publishMsg(msg_id::CTRL_SETPOINT, &msg);
        }
        else{
            _traj_queue.front()->publish();
        }
    }

    int usrcmd_trajectoryCommander(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status :\r\n");
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            TrajectoryCommander::getInstance().debug();
            return 0;
        }

        return 0;
    }
}
