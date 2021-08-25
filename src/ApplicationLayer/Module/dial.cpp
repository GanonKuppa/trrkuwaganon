#include "dial.h"
#include "debugLog.h"

Dial::Dial() :
_enable(false),
_dial_position(0),
_division_num(8),
_duty(0.0f),
_p_gain(0.0f),
_i_gain(0.0f),
_i_limit(0.0f),
_limit(0.0f)
{   
    _ang_pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
}

void Dial::setEnable(bool enable){
    _enable = enable;
}

bool Dial::getEnable(){
    return _enable;
}

void Dial::update(){
    
    if(!_enable) return;
    float angle_rad = getAngle();
    float angle_deg = angle_rad * RAD2DEG;
    _dial_position = (int)((angle_deg / 360.0) * (float)_division_num);
    
    if(_division_num == 0) return;
    else{
        
        float target_ang = DEG2RAD * 360.0f * ((float)_dial_position + 0.5f) / (float)_division_num;
        
        _ang_pidf.set(_p_gain, _i_gain, 0.0f, 0.0f);
        _ang_pidf.setSaturationEnable(true);
        _ang_pidf.setSaturation(_limit);
        _ang_pidf.setIntegralSaturationEnable(true);
        _ang_pidf.setIntegralSaturation(_i_limit);
        _ang_pidf.update(target_ang, angle_rad);
        _duty = _ang_pidf.getControlVal();
        
    }
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

void Dial::setPiGain(float p_gain, float i_gain, float i_limit, float limit){
    _p_gain = p_gain;
    _i_gain = i_gain;
    _i_limit = i_limit;
    _limit = limit;
}

void Dial::reset() {
    //wodo.resetTireAng();
    _dial_position = 0;
    _ang_pidf.reset();
}

void Dial::debug(){    
    PRINTF_ASYNC("  enable       : %d \n", _enable);
    PRINTF_ASYNC("  dial_pos     : %d \n", _dial_position);
    PRINTF_ASYNC("  div_num      : %d \n", _division_num);
    PRINTF_ASYNC("  duty         : %f \n", _duty);    
}
