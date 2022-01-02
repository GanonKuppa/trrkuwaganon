#pragma once

#include <algorithm>
#include <cmath>
#include <cfloat>


class PidfController {
  public:
    PidfController() :
    _Kp(0.0f),
    _Ki(0.0f),
    _Kd(0.0f),
    _F(0.0f),
    _e_p0(0.0f),
    _e_i0(0.0f),
    _e_d0(0.0f),
    _u_k0(0.0f),
    _e_p1(0.0f),
    _e_i1(0.0f),
    _e_d1(0.0f),
    _u_k1(0.0f),
    _enable(true),
    _integral_saturation_enable(true),
    _integral_saturation(-1.0f),
    _saturation_enable(true),
    _saturation(-1.0f),
    _delta_t(0.001f)
    {
        
    }

    virtual void update(float target, float observed_val){
        if(!_enable) return;
                
        float error = target - observed_val;
        _updateController(error);
    }    

    float getControlVal(){
        if(_enable) return _u_k0;
        else return 0.0f;
    }

    float getPVal(){
        return _Kp * _e_p0;
    }

    float getIVal(){
        return _Ki * _e_i0;
    }

    float getDVal(){
        return _Kd * _e_d0;
    }

    void set(float Kp, float Ki, float Kd, float F) {
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
        _F = F;
    }

    void setIntegralSaturation(float integral_saturation) {
        _integral_saturation = integral_saturation;
        if(_integral_saturation_enable && _integral_saturation > 0.0f && _Ki > 0.0f){
            _e_i0 = std::clamp<float>(_e_i0, -std::fabs(_integral_saturation)/_Ki, std::fabs(_integral_saturation)/_Ki);
            _e_i1 = std::clamp<float>(_e_i0, -std::fabs(_integral_saturation)/_Ki, std::fabs(_integral_saturation)/_Ki);
        }
    }

    void setEnable(bool enable) {
        _enable = enable;
    }

    void setSaturationEnable(bool saturation_enable) {
        _saturation_enable = saturation_enable;
    }

    void setSaturation(float saturation){
        _saturation = saturation;
        if(_saturation > 0.0f &&  _saturation_enable){
            _u_k0 = std::clamp<float>(_u_k0, -std::fabs(_saturation), std::fabs(_saturation));
            _u_k1 = std::clamp<float>(_u_k1, -std::fabs(_saturation), std::fabs(_saturation));
        }
    }

    void setIntegralSaturationEnable(bool enable){
        _integral_saturation_enable = enable;
    }

    void setDeltaT(float delta_t){
        _delta_t = delta_t;
    }

    float getError(){
        return _e_p0;
    }

    void reset() {
        _e_p0 = 0.0f;
        _e_i0 = 0.0f;
        _e_d0 = 0.0f;
        _u_k0 = 0.0f;
        _e_p1 = 0.0f;
        _e_i1 = 0.0f;
        _e_d1 = 0.0f;
        _u_k1 = 0.0f;
    }

    bool engaged(){
        if (std::fabs(_u_k0) < FLT_EPSILON) return false;
        else return true;
    }

  protected:
    float _Kp;
    float _Ki;
    float _Kd;
    float _F;
    float _e_p0;
    float _e_i0;
    float _e_d0;
    float _u_k0;
    float _e_p1;
    float _e_i1;
    float _e_d1;
    float _u_k1;
    
    bool _enable;

    bool _integral_saturation_enable;
    float _integral_saturation;
    
    bool _saturation_enable;
    float _saturation;
    
    float _delta_t;

    void _updateController(float error){
        _e_p0 = error;
        if(_integral_saturation_enable && _integral_saturation > 0.0f && _Ki > 0.0f){
            _e_i0 = std::clamp<float>(_e_p0 * _delta_t + _e_i1, -std::fabs(_integral_saturation) / _Ki, std::fabs(_integral_saturation) / _Ki);
        }
        else {
            _e_i0 = _e_p0 * _delta_t + _e_i1;
        }

        _e_d0 = _F * _e_d1 + (1.0f - _F) * (_e_p0 - _e_p1) / _delta_t;

        if(_saturation_enable && _saturation > 0.0f){
            _u_k0 = std::clamp<float>(_Kp * _e_p0 + _Ki * _e_i0 + _Kd * _e_d0, -std::fabs(_saturation), std::fabs(_saturation));
        }
        else{
            _u_k0 = _Kp * _e_p0 + _Ki * _e_i0 + _Kd * _e_d0;
        }

        _e_p1 = _e_p0;
        _e_i1 = _e_i0;
        _e_d1 = _e_d0;
        _u_k1 = _u_k0;

    }
};

class AngPidfController : public PidfController {
  public:
    AngPidfController() : 
    PidfController()
    {

    }

    void update(float target, float observed_val) {
        if(!_enable) return;
        
        float ang_diff = target - observed_val;
        while(ang_diff >  PI) ang_diff -= 2.0f * PI;
        while(ang_diff < -PI) ang_diff += 2.0f * PI;
        _updateController(ang_diff);
    };
  private:
    const float PI = 3.141592653589f;

};

class WallPidfController : public PidfController {
  public:

    WallPidfController() : 
    PidfController()
    {

    }    

    void update(float center_dist_r, float center_dist_l, bool is_r_enable, bool is_l_enable) {
        if(!_enable) return;

        if(!is_r_enable && !is_l_enable) {
            reset();
            return;
        }
        float error = 0.0f;   
        
        if(is_r_enable && is_l_enable) error = (center_dist_r - center_dist_l) / 2.0f;
        else if(is_r_enable) error = center_dist_r;
        else if(is_l_enable) error = - center_dist_l;
        else error = 0.0f;
        _updateController(error);
    };
    
};
