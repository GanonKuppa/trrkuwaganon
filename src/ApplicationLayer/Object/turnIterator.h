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
        _yaw = 0.0f;
        _yawrate = 0.0f;
        _yawacc = 0.0f;
    }
    
    void next(){
        float _yawrate_now = mollifier_f(_t_molli * _v, _a, _b, _c);
        if(_yawrate_now < 0.01745f){ // 0.01745 rad/s = 1deg/s
            _t_molli += _delta_t;
            next();
        }

        _yawacc = (_yawrate_now - _yawrate) / _delta_t;
        _yawrate = _yawrate_now;
        _yaw = _yawrate * _delta_t;

        float beta_pre = _beta;
        float t_pre = _t;
        float k1 = _rk4_f(t_pre                , beta_pre                     );
        float k2 = _rk4_f(t_pre + _delta_t*0.5f, beta_pre + _delta_t * 0.5f*k1);
        float k3 = _rk4_f(t_pre + _delta_t*0.5f, beta_pre + _delta_t * 0.5f*k2);
        float k4 = _rk4_f(t_pre + _delta_t     , beta_pre + _delta_t * k3     );
        _beta = beta_pre + (k1 + 2.0f*k2 + 2.0f*k3 + k4) * _delta_t / 6.0f;    
        _t += _delta_t;
    };
    float getYawrate(){
        return _yawrate;
    };
    float getYawAcc(){
        return _yawacc;
    };
    float getBeta(){
        return _beta;
    };
    float getElapsedTime(){
        return _t;
    };

    bool isEnd(){
        // 0.01745 rad/s = 1deg/s
        return (_yaw > _target_ang * 0.99f) && (_yawrate < FLT_EPSILON) && std::fabs(_beta) < 0.01745f;
    };
  private:
    float _delta_t;
    float _v;
    float _cp;
    float _target_ang;

    float _a;
    float _b;
    float _c;    

    float _t_molli;
    float _t;
    float _beta;
    float _yaw;
    float _yawrate;
    float _yawacc;

    float _rk4_f(float t, float y){
        return -_cp * y / _v - _v * mollifier_f(t * _v, _a, _b, _c);
    }
};
