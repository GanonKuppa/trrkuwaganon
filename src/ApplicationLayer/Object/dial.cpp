#include "dial.h"
#include "debugLog.h"

#include <cmath>

Dial::Dial() :
_enable(false),
_dial_position(0),
_division_num(8),
_duty(0.0f),
_p_gain(0.0f),
_i_gain(0.0f),
_i_limit(0.0f),
_limit(0.0f),
_ang_origin(0.0f),
_same_pos_time(0.0f),
_delta_t(0.001f),
_target_ang(0.0f),
_ang(0.0f)
{   
    _ang_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
}

void Dial::setEnable(bool enable){
    _enable = enable;
}

void Dial::setDeltaT(float delta_t){    
    _delta_t = delta_t;
    _ang_pidf.setDeltaT(delta_t);
}

bool Dial::getEnable(){
    return _enable;
}

void Dial::update(){
    
    if(!_enable || _division_num == 0){
        return;
    } 

    float ang_div_half_offset = 0.5f * 2.0f * PI / (float)_division_num;
    _ang = fmodf(getAngle() + ang_div_half_offset - _ang_origin + 2.0f * PI, 2.0f * PI);

    float angle_deg = _ang * RAD2DEG;
    
    uint8_t dial_position_pre = _dial_position;
    _dial_position = (int)((angle_deg / 360.0) * (float)_division_num);
    if(dial_position_pre == _dial_position){
        _same_pos_time += _delta_t;
    }
    else{
        _same_pos_time = 0.0f;
    }

        
    _target_ang = DEG2RAD * 360.0f * ((float)_dial_position + 0.5f) / (float)_division_num;
    
    _ang_pidf.set(_p_gain, _i_gain, 0.0f, 0.0f);
    _ang_pidf.setSaturationEnable(true);
    _ang_pidf.setSaturation(_limit);
    _ang_pidf.setIntegralSaturationEnable(true);
    _ang_pidf.setIntegralSaturation(_i_limit);
    _ang_pidf.update(_target_ang, _ang);
    if(std::fabs(getVelocity()) > 0.1f){
        _ang_pidf.reset();
    }
    _duty = _ang_pidf.getControlVal();
        
    
}

float Dial::getDuty(){
    return _duty;
}

void Dial::incrementPosition() {
    _dial_position = (_dial_position + (_division_num + 1)) % _division_num;
}

void Dial::decrementPosition() {
    _dial_position = (_dial_position + (_division_num - 1)) % _division_num;
}

void Dial::setDivisionNum(uint8_t num){
    _division_num = num;    
}

uint8_t Dial::getDialPosition(){
    return _dial_position;
}

uint8_t Dial::getDivisionNum(){
    return _division_num;
}

float Dial::getSamePosTime(){
    return _same_pos_time;
}

void Dial::setPiGain(float p_gain, float i_gain, float i_limit, float limit){
    _p_gain = p_gain;
    _i_gain = i_gain;
    _i_limit = i_limit;
    _limit = limit;
}

void Dial::reset() {    
    _dial_position = 0;
    _ang_pidf.reset();
    _ang_origin = getAngle();
    _same_pos_time = 0.0f;
}

void Dial::debug(){    
    PRINTF_ASYNC("  enable       : %d \n", _enable);
    PRINTF_ASYNC("  dial_pos     : %d \n", _dial_position);
    PRINTF_ASYNC("  div_num      : %d \n", _division_num);
    PRINTF_ASYNC("  duty         : %f \n", _duty);
    PRINTF_ASYNC("  target_ang   : %f \n", _target_ang * RAD2DEG);
    PRINTF_ASYNC("  ang          : %f \n", _ang * RAD2DEG);
}
