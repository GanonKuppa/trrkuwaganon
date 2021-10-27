#include "trajectoryCommander.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"

namespace module {

    TrajectoryCommander::TrajectoryCommander() :
    _x(0.045f),
    _v_x(0.0f),
    _a_x(0.0f),
    _y(0.045f),
    _v_y(0.0f),
    _a_y(0.0f),
    _v_xy_body(0.0f),
    _a_xy_body(0.0f),
    _yaw(90.0f * 3.14159265f / 180.0f),
    _yawrate(0.0f),
    _yawacc(0.0f),
    _beta(0.0f),
    _beta_dot(0.0f),
    _traj_type(ETrajType::NONE),
    _turn_type(ETurnType::NONE),
    _turn_dir(ETurnDir::NO_TURN),
    _lock_guard(false)    
    {
        setModuleName("TrajectoryCommander");
    };

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
        _v_x = 0.0f;
        _a_x = 0.0f;
        _y = y;
        _v_y = 0.0f;
        _a_y = 0.0f;
        _v_xy_body = 0.0f;        
        _a_xy_body = 0.0f;
        _yaw = yaw;
        _yawrate = 0.0f;
        _yawacc = 0.0f;
        _beta = 0.0f;
        _beta_dot = 0.0f;
    }

    void TrajectoryCommander::update0(){
        if(_lock_guard) return;
        
        if(!_traj_queue.empty()){
            if(_traj_queue.front()->isEnd()){
                _x = _traj_queue.front()->getEndX();
                _y = _traj_queue.front()->getEndY();
                _yaw = _traj_queue.front()->getEndYaw();
                _traj_queue.pop_front();
            }
        }        
        _publish();
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
        return 0;
    }
}
