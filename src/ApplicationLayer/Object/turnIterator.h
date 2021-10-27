#pragma once

#include <cmath>
#include <cfloat>

#include "mollifier.h"

class TurnIterator{
  public:
    TurnIterator(float int_molli_c, float shape_factor, float path_length, float target_ang, float v, float cp, float delta_t){
        _target_ang = target_ang;
        _v = v;
        _b = path_length;
        _c = shape_factor;
        _cp = cp;
        float S = int_molli_c;
        _a = 1.0f / (target_ang / _b / S);
        _delta_t = delta_t;
        _t_molli = 0.0f;
        _t = 0.0f;
        _beta = 0.0f;
        _beta_dot = 0.0f;
        _beta_abs_max = 0.0f;
        _yaw = 0.0f;
        _yawrate = 0.0f;
        _yawacc = 0.0f;
    }
    
    void next(){
        float yawrate_pre = _yawrate;
        float yawrate_pre_plus_half_dt = _v * mollifier_f((_t_molli + _delta_t * 0.5f) * _v, _a, _b, _c);
        float yawrate_now = _v * mollifier_f(_t_molli * _v, _a, _b, _c);
        float yawrate_next = _v * mollifier_f((_t_molli + _delta_t) * _v, _a, _b, _c);
        _t_molli += _delta_t;
        if(yawrate_now < 0.01745f && _yaw < 0.0001745f){ // 0.01745 rad/s = 1deg/s            
            _yawrate = yawrate_now;
            next();
        }

        _yawacc = (yawrate_next - yawrate_pre) / (2.0f * _delta_t); // 中心差分による微分計算
        _yaw += _rk4_yaw_delta(yawrate_pre, yawrate_pre_plus_half_dt, yawrate_now);
        _yawrate = yawrate_now;

        float beta_pre = _beta;
        _beta_dot = _rk4_beta_f(beta_pre, yawrate_pre);
        _beta += _rk4_beta_delta(beta_pre, yawrate_pre, yawrate_pre_plus_half_dt, yawrate_now);
        if(_beta_abs_max < std::fabs(_beta)){
            _beta_abs_max = std::fabs(_beta);
        }
        _t += _delta_t;
    };

    float getYawrate(){
        return _yawrate;
    };

    float getYawAcc(){
        return _yawacc;
    };

    float getYaw(){
        return _yaw;
    }

    float getBeta(){
        return _beta;
    };

    float getBetaDot(){
        return _beta_dot;
    }

    float getBetaAbsMax(){
        return _beta_abs_max;
    }

    float getElapsedTime(){
        return _t;
    };

    float getTargetAng(){
        return _target_ang;
    }
    
    bool isEnd(){
        // 0.001745f rad = 0.1deg 0.01745 rad/s = 1deg/s
        return (std::fabs(_yaw - _target_ang) < 0.001745f) && (_yawrate < 0.001745f) && std::fabs(_beta) < 0.01745f;
    };
  private:
    float _delta_t;
    float _v;
    float _cp;
    float _target_ang;
    float _start_ang;

    float _a;
    float _b;
    float _c;    

    float _t_molli;
    float _t;
    float _beta;
    float _beta_dot;
    float _beta_abs_max;
    float _yaw;
    float _yawrate;
    float _yawacc;

    float _move_x;
    float _move_y;

    float _rk4_beta_f(float beta, float yawrate){
        return -_cp * beta / _v - yawrate;
    }

    float _rk4_beta_delta(float beta_pre, float yawrate_pre, float yawrate_pre_plus_half_dt, float yawrate_now){
        float k1 = _rk4_beta_f(beta_pre                       , yawrate_pre);
        float k2 = _rk4_beta_f(beta_pre + _delta_t * 0.5f * k1, yawrate_pre_plus_half_dt);
        float k3 = _rk4_beta_f(beta_pre + _delta_t * 0.5f * k2, yawrate_pre_plus_half_dt);
        float k4 = _rk4_beta_f(beta_pre + _delta_t * k3       , yawrate_now);
        return (k1 + 2.0f*k2 + 2.0f*k3 + k4) * _delta_t / 6.0f;
    }

    float _rk4_yaw_delta(float yawrate_pre, float yawrate_pre_plus_dt, float yawrate_now){                
        float k1 = yawrate_pre;
        float k2 = yawrate_pre_plus_dt;
        float k3 = k2;
        float k4 = yawrate_now;
        return (k1 + 2.0f*k2 + 2.0f*k3 + k4) * _delta_t / 6.0f;
    }

};
