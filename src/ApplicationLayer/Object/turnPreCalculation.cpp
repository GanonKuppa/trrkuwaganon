#include "turnPreCalculation.h" 

#include <cmath>

#include "turnIterator.h"

#include "mollifier.h"


TurnPreCalculation::TurnPreCalculation(float shape_factor,
        float path_length,
        float target_ang,
        float start_ang,
        float start_x,
        float start_y,
        float end_x,
        float end_y,
        float v,
        float cp,
        float delta_t){

    _pre_dist = 0.0f;
    _fol_dist = 0.0f;
    _shape_factor = shape_factor;
    _path_length = path_length;
    _target_ang = target_ang;
    _start_ang = start_ang;
    _start_x = start_x;
    _start_y = start_y;
    _end_x = end_x;
    _end_y = end_y;
    _v = v;
    _cp = cp;
    _delta_t = delta_t;
    _end_time = 0.0f;
    _move_x = 0.0f;
    _move_y = 0.0f;
    _beta_abs_max = 0.0f;

	_int_molli_c = intMollifier_f_tbl(_shape_factor);
    
	constexpr float TURN_180_ANG = 3.14159265f;
    if(std::fabs(target_ang - TURN_180_ANG) < 0.01745f){ // 0.01745rad = 1度        
        _init180();
    }
    else{
        _init();
    }    
}

TurnPreCalculation::TurnPreCalculation(){
}


std::unique_ptr<TurnIterator> TurnPreCalculation::generateTurnIterator(){
    return std::make_unique<TurnIterator>(_int_molli_c, _shape_factor, _path_length, _target_ang, _v, _cp, _delta_t);
}
    
float TurnPreCalculation::getTrajEndTime(){
    return _end_time;
}


float TurnPreCalculation::getPreDist(){
    return _pre_dist;
}

float TurnPreCalculation::getFolDist(){
    return _fol_dist;
}

float TurnPreCalculation::getBetaAbsMax(){
    return _beta_abs_max;
};

void TurnPreCalculation::_init(){
    float delta_t_for_calc_dist = 0.0001f;
    auto turn_itr = TurnIterator(_int_molli_c, _shape_factor, _path_length, _target_ang, _v, _cp, delta_t_for_calc_dist);
    float x = _start_x;
    float y = _start_y;
    float yaw = _start_ang;
    float yawrate = 0.0f;
    float beta = 0.0f;
    float yaw_pre = _start_ang;
    float beta_pre = 0.0f;
    float yawrate_pre = 0.0f;
    
    while(!turn_itr.isEnd()){
        yawrate = turn_itr.getYawrate();
        beta = turn_itr.getBeta();      
        yaw = _start_ang + turn_itr.getYaw();

        if(yawrate_pre > 0.0087f){ // 0.087rad = 5deg
            x += _v * cosf(yaw_pre + beta_pre) * sinf(yawrate_pre * delta_t_for_calc_dist * 0.5f) / (yawrate_pre * 0.5f);
            y += _v * sinf(yaw_pre + beta_pre) * sinf(yawrate_pre * delta_t_for_calc_dist * 0.5f) / (yawrate_pre * 0.5f);
        }                
        else{
            x += _v * cosf(yaw_pre + beta_pre) * delta_t_for_calc_dist;
            y += _v * sinf(yaw_pre + beta_pre) * delta_t_for_calc_dist;
        }
        yaw_pre = yaw;
        beta_pre = beta;
        yawrate_pre = yawrate;
        turn_itr.next();     
    }
    _move_x = x - _start_x;
    _move_y = y - _start_y;


    float X = _end_x - _start_x - _move_x;
    float Y = _end_y - _start_y - _move_y;
    float end_ang = _start_ang + _target_ang;
    float sin_start_ang = sinf(_start_ang);
    float cos_start_ang = cosf(_start_ang);
    float sin_end_ang = sinf(end_ang);
    float cos_end_ang = cosf(end_ang);
        
    _pre_dist = (sin_end_ang   * X - cos_end_ang   * Y) / (cos_start_ang * sin_end_ang   - cos_end_ang   * sin_start_ang);
    _fol_dist = (sin_start_ang * X - cos_start_ang * Y) / (cos_end_ang   * sin_start_ang - cos_start_ang * sin_end_ang  );
    _end_time = turn_itr.getElapsedTime();
    _beta_abs_max = turn_itr.getBetaAbsMax();
}

void TurnPreCalculation::_init180(){
    float error_y = 1.0f; // 大きな値
    float path_length_min = 0.01f; 
    float path_length_max = 0.2f;

    while(std::fabs(error_y) > 0.0001f){
        float pl = 0.5 * (path_length_min + path_length_max);
        _path_length = pl;
        _init();
        error_y = _move_y - _end_y;        
        if(error_y > 0.0f){
            path_length_max = pl;
        }            
        else{
            path_length_min = pl;
        }
        if(std::fabs(path_length_max - path_length_min) < 0.00001f){ // 2分探索範囲がfloatの分解能以下になったときの例外処理
            break;
        }
    }
    _pre_dist = 0.027f; // 180度ターンのみ前距離は任意の値で成立し得るので壁にぶつからない値をセット
    _fol_dist = _pre_dist + _move_x;
}
