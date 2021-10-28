#include "trajectory.h"

#include <math.h>

// Msg
#include "vehiclePositionMsg.h"
#include "ctrlSetpointMsg.h"
#include "msgBroker.h"

// Mod
#include "trajectoryInitializer.h"

//-- BaseTrajectory
BaseTrajectory::BaseTrajectory() : 
    _x(0.0f),
    _v_x(0.0f),
    _a_x(0.0f),
    _y(0.0f),
    _v_y(0.0f),
    _a_y(0.0f),
    _v_xy_body(0.0f),
    _a_xy_body(0.0f),
    _yaw(0.0f),
    _yawrate(0.0f),
    _yawacc(0.0f),
    _beta(0.0f),
    _beta_dot(0.0f),
    _delta_t(0.001f),
    _x_0(0.0f),
    _y_0(0.0f),
    _yaw_0(0.0f),
    _cumulative_dist(0.0f),
    _cumulative_yaw(0.0f),
    _cumulative_t(0.0f),
    _traj_type(ETrajType::NONE),
    _turn_type(ETurnType::NONE),
    _turn_dir(ETurnDir::NO_TURN)
{

}

void BaseTrajectory::setInitPos(float x, float y, float yaw) {
    _x = x;
    _y = y;
    _yaw = yaw;
    _x_0 = x;
    _y_0 = y;
    _yaw_0 = yaw;
}

float BaseTrajectory::getNecessaryTime() {
        _delta_t = 0.01f;
        float necessary_time = 0.0f;
        while(!isEnd()) {
            update();
            necessary_time += _delta_t;
        }
        return necessary_time;
    }

void BaseTrajectory::update(){
        _cumulative_dist += _delta_t * _v_xy_body;

        _cumulative_yaw += _delta_t * _yawrate;
        _yaw += _delta_t * _yawrate;
        constexpr float PI = 3.14159265f;
        _yaw = fmod(_yaw + 2.0f*PI, 2.0f*PI);

        _x += _delta_t * _v_x;
        _y += _delta_t * _v_y;

        _v_x = _v_xy_body * cosf(_yaw + _beta);
        _v_y = _v_xy_body * sinf(_yaw + _beta);

        _v_xy_body += _delta_t * _a_xy_body;
        _yawrate += _delta_t * _yawacc;

        _a_x = _a_xy_body * cosf(_yaw + _beta) - _v_xy_body * (_yawrate + _beta_dot) * sinf(_yaw + _beta);
        _a_y = _a_xy_body * sinf(_yaw + _beta) + _v_xy_body * (_yawrate + _beta_dot) * cosf(_yaw + _beta);

        _cumulative_t += _delta_t;
};

void BaseTrajectory::publish(){
    CtrlSetpointMsg msg;
    msg.x = _x;
    msg.v_x = _v_x;
    msg.a_x = _a_x;
    msg.y = _y;
    msg.v_y = _v_y;
    msg.a_y = _a_y;
    msg.v_xy_body = _v_xy_body;
    msg.a_xy_body = _a_xy_body;
    msg.yaw = _yaw;
    msg.yawrate = _yawrate;        
    msg.yawacc = _yawacc;
    msg.beta = _beta;
    msg.beta_dot = _beta_dot;
    msg.traj_type = _traj_type;
    msg.turn_type = _turn_type;
    msg.turn_dir = _turn_dir;
    publishMsg(msg_id::CTRL_SETPOINT, &msg);
};

//-- StraightTrajectory
StraightTrajectory::StraightTrajectory(ETurnType turn_type, float target_dist, float v_0) : 
    BaseTrajectory(){
    _a_acc = 0.0;
    _a_dec = 0.0;
    _target_dist = target_dist;
    _v_end = v_0;
    _v_max = v_0;
    _v_0 = v_0;
    _v_xy_body = v_0;
    _a_xy_body = 0.0f;
    _traj_type = ETrajType::STRAIGHT;
    if(isTurnStraight(turn_type)){
        _turn_type = turn_type;
    }
    else{
        _turn_type = ETurnType::STRAIGHT;
    }
    _turn_dir = ETurnDir::NO_TURN;
}

StraightTrajectory::StraightTrajectory(ETurnType turn_type, float target_dist, float v_0, float v_max, float v_end, float a_acc, float a_dec) :
    BaseTrajectory(){
    _a_acc = a_acc;
    _a_dec = a_dec;
    _target_dist = target_dist;
    _v_end = v_end;
    _v_min = 0.05f;
    _v_max = v_max;
    _v_0 = v_0;
    _v_xy_body = v_0;
    _a_xy_body = a_acc;

    _traj_type = ETrajType::STRAIGHT;
    if(isTurnStraight(turn_type)){
        _turn_type = turn_type;
    }
    else{
        _turn_type = ETurnType::STRAIGHT;
    }
    _turn_dir = ETurnDir::NO_TURN;
}

float StraightTrajectory::getEndX(){
    return _x_0 + _target_dist * cosf(_yaw_0);
}
    
float StraightTrajectory::getEndY(){
    return _y_0 + _target_dist * sinf(_yaw_0);
}

float StraightTrajectory::getEndYaw(){
    return _yaw_0;
}

float StraightTrajectory::getNecessaryTime(){
    _delta_t = 0.01;
    float necessary_time = 0.0f;
    while(_cumulative_dist < _target_dist) {
        update();
        necessary_time += _delta_t;
    }
    return necessary_time;        
}

void StraightTrajectory::update() {
    BaseTrajectory::update();

    if( (_a_acc == 0.0f && _a_dec == 0.0f)||
        (_v_max == _v_0 && _v_max == _v_end)
    ) {
        _a_acc = 0.0f;
        _a_dec = 0.0f;
        _a_xy_body = 0.0f;
        return;
    }

    float x_bre = 0.0f;
    float dist = _target_dist;
    if(_target_dist > (0.09f + 0.045f) && _turn_type == ETurnType::STRAIGHT_CENTER_EDGE){
        dist = _target_dist - 0.045f;
    }else{
        dist = _target_dist - 0.0025f;
    }

    if (_a_dec != 0.0f){
        x_bre = (_v_xy_body * _v_xy_body - _v_end * _v_end) / (2.0f * _a_dec);
    }

    if (x_bre > (dist - _cumulative_dist)){
        _a_xy_body = -_a_dec;
    }

    if (_v_xy_body >= _v_max && _a_xy_body > 0.0f) {
        _v_xy_body = _v_max;
        _a_xy_body = 0.0f;
    }

    if (_v_xy_body < _v_end && _a_xy_body < 0.0f) {
        _v_xy_body = _v_end;
        _a_xy_body = 0.0f;
    }

    if(_v_xy_body <= _v_min && _a_xy_body < 0.0f){
        _v_xy_body = _v_min;
        _a_xy_body = 0.0f;
    }
}
    
bool StraightTrajectory::isEnd() {
    VehiclePositionMsg pos_msg;
    copyMsg(msg_id::VEHICLE_POSITION, &pos_msg);
    float res_dist = -1.0f;//_calcResidualDist(pos_msg.x, pos_msg.y);

    if (_cumulative_dist >= _target_dist && res_dist <= 0.0f) {
        _x = getEndX();
        _y = getEndY();
        _yaw = getEndYaw();
        return true;
    } else {
        return false;
    }
}
    
//-- SpinTurnTrajectory    
SpinTurnTrajectory::SpinTurnTrajectory(float target_cumulative_yaw, float abs_yawrate_max, float abs_yawacc) : 
    BaseTrajectory(){
    
    _traj_type = ETrajType::SPINTURN;
    _turn_type = ETurnType::SPINTURN;
    _turn_dir = (ETurnDir)((target_cumulative_yaw>0)-(target_cumulative_yaw<0));


    _target_cumulative_yaw = target_cumulative_yaw;
    _yawacc = (float)_turn_dir * abs_yawacc;

    _target_cumulative_yaw = target_cumulative_yaw;
    _abs_yawacc = abs_yawacc;
    _abs_yawrate_max = abs_yawrate_max;    
    _abs_yawrate_min = 0.349f;  // 0.349rad = 20deg/sec    
}

float SpinTurnTrajectory::getEndX() {
    return _x_0;
}

float SpinTurnTrajectory::getEndY() {
    return _y_0;
}

float SpinTurnTrajectory::getEndYaw() {
    constexpr float PI = 3.14159265f;
    return fmod(_yaw_0 + _target_cumulative_yaw + 2.0f * PI, 2.0f * PI);
}

void SpinTurnTrajectory::update() {        
    BaseTrajectory::update();
    
    float abs_yaw_bre = 0.0f;
    float abs_target_cumulative_yaw = std::fabs(_target_cumulative_yaw);
    if(abs_target_cumulative_yaw > 0.174f){ // 0.174rad = 10deg, 0.0174*3.0 rad = 2deg
        abs_target_cumulative_yaw -= 0.0174f * 2.0f;
    }


    if (_abs_yawacc != 0.0f){
        abs_yaw_bre = (_yawrate * _yawrate - _abs_yawrate_min * _abs_yawrate_min) / (2.0f * _abs_yawacc);
    }

    if (abs_yaw_bre >= (abs_target_cumulative_yaw - std::fabs(_cumulative_yaw))){
        _yawacc = - (float)_turn_dir * _abs_yawacc;
    }

    if (std::fabs(_yawrate) > _abs_yawrate_max) {
        _yawrate = (float)_turn_dir * _abs_yawrate_max;
        _yawacc = 0.0f;
    }

    if (std::fabs(_yawrate) <= _abs_yawrate_min && _yawacc * (float)_turn_dir < 0.0f){
        _yawrate = (float)_turn_dir * _abs_yawrate_min;
        _yawacc = 0.0f;
    }
}

bool SpinTurnTrajectory::isEnd() {
    if(_target_cumulative_yaw == 0.0f){
        return true;
    } 

    if (std::fabs(_cumulative_yaw) >= std::fabs(_target_cumulative_yaw)) {
        _x = getEndX();
        _y = getEndY();
        _yaw = getEndYaw();
        return true;
    } else{
        return false;
    }            
}

//-- StopTrajectory
StopTrajectory::StopTrajectory(float stop_time){
    _traj_type = ETrajType::STOP;
    _turn_type = ETurnType::STOP;
    _turn_dir = ETurnDir::NO_TURN;
    _stop_time = stop_time;
}

StopTrajectory::StopTrajectory(float stop_time, float x, float y, float yaw) : 
    BaseTrajectory(){
    _x = x;
    _y = y;
    _yaw = yaw;
    _x_0 = x;
    _y_0 = y;
    _yaw_0 = yaw;
    _traj_type = ETrajType::STOP;
    _turn_type = ETurnType::STOP;
    _turn_dir = ETurnDir::NO_TURN;
    _stop_time = stop_time;
}

float StopTrajectory::getEndX() {
    return _x_0;
}

float StopTrajectory::getEndY() {
    return _y_0;
}

float StopTrajectory::getEndYaw() {
    return _yaw_0;
}

float StopTrajectory::getNecessaryTime(){
    return _stop_time;
};


void StopTrajectory::update() {
    _cumulative_t += _delta_t;
}

bool StopTrajectory::isEnd() {
    if(_cumulative_t >= _stop_time){
        _x = getEndX();
        _y = getEndY();
        _yaw = getEndYaw();
        return true;
    }else{
        return false;
    }
}

//-- CurveTrajectory
CurveTrajectory::CurveTrajectory(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir) : 
    BaseTrajectory(){
    _turn_iterator = module::TrajectoryInitializer::getInstance().generateTurnIterator(param_set, turn_type);
    _yaw_pre = 0.0f;
    _beta_pre = 0.0f;
    _yawrate_pre = 0.0f;

    _traj_type = ETrajType::CURVE;
    _turn_type = turn_type;
    _param_set = param_set;
    _turn_dir = turn_dir;
    _v_xy_body = module::TrajectoryInitializer::getInstance().getV(param_set, turn_type);
}

float CurveTrajectory::getEndX(){
    float move_x = module::TrajectoryInitializer::getInstance().getMoveX(_param_set, _turn_type);
    float move_y = module::TrajectoryInitializer::getInstance().getMoveY(_param_set, _turn_type) * (float)(_turn_dir);
    return _x_0 + move_x * cosf(_yaw_0) - move_y * sinf(_yaw_0);
}

float CurveTrajectory::getEndY(){
    float move_x = module::TrajectoryInitializer::getInstance().getMoveX(_param_set, _turn_type);
    float move_y = module::TrajectoryInitializer::getInstance().getMoveY(_param_set, _turn_type) * (float)(_turn_dir);
    return _y_0 + move_x * sinf(_yaw_0) + move_y * cosf(_yaw_0);
}

float CurveTrajectory::getEndYaw(){
    float target_ang = (float)(_turn_dir) * _turn_iterator->getTargetAng();
    constexpr float PI = 3.14159265f;
    return fmod(_yaw_0 + target_ang + 2.0f * PI, 2.0f * PI);
}

float CurveTrajectory::getNecessaryTime(){
    return module::TrajectoryInitializer::getInstance().getTrajEndTime(_param_set, _turn_type);
}

void CurveTrajectory::update(){
    _v_x = _v_xy_body * cosf(_yaw + _beta);
    _v_y = _v_xy_body * sinf(_yaw + _beta);

    _a_x = _a_xy_body * cosf(_yaw + _beta) - _v_xy_body * (_yawrate + _beta_dot) * sinf(_yaw + _beta);
    _a_y = _a_xy_body * sinf(_yaw + _beta) + _v_xy_body * (_yawrate + _beta_dot) * cosf(_yaw + _beta);

    _yawacc = _turn_iterator->getYawAcc() * (float)(_turn_dir);
    _yawrate = _turn_iterator->getYawrate() * (float)(_turn_dir);
    _beta = _turn_iterator->getBeta() * (float)(_turn_dir);
    _beta_dot = _turn_iterator->getBetaDot() * (float)(_turn_dir);
    
    _yaw = _yaw_0 + _turn_iterator->getYaw() * (float)(_turn_dir);
    constexpr float PI = 3.14159265f;
    _yaw = fmod(_yaw + 2.0f * PI, 2.0f * PI);


    if(_yawrate_pre > 0.0087f){ // 0.087rad = 5deg
        _x += _v_xy_body * cosf(_yaw_pre + _beta_pre) * sinf(_yawrate_pre * _delta_t * 0.5f) / (_yawrate_pre * 0.5f);
        _y += _v_xy_body * sinf(_yaw_pre + _beta_pre) * sinf(_yawrate_pre * _delta_t * 0.5f) / (_yawrate_pre * 0.5f);
    }                
    else{
        _x += _v_xy_body * cosf(_yaw_pre + _beta_pre) * _delta_t;
        _y += _v_xy_body * sinf(_yaw_pre + _beta_pre) * _delta_t;
    }
    _yaw_pre = _yaw;
    _beta_pre = _beta;
    _yawrate_pre = _yawrate;

    _cumulative_dist += _delta_t * _v_xy_body;
    _cumulative_yaw += _delta_t * _yawrate;
    _cumulative_t += _delta_t;

    _turn_iterator->next();
}

bool CurveTrajectory::isEnd(){
    if(_turn_iterator->isEnd()){
        _x = getEndX();
        _y = getEndY();
        _yaw = getEndYaw();
        return true;
    }else{
        return false;
    }    
}
