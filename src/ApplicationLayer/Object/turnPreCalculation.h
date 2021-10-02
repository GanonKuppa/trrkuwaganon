#pragma once

#include <memory>

#include "turnIterator.h"

class TurnPreCalculation{
  public:
    TurnPreCalculation(float shape_factor, 
                       float path_length, 
                       float target_ang,
                       float start_ang,
                       float start_x,
                       float start_y,
                       float end_x,
                       float end_y,
                       float v, 
                       float cp, 
                       float delta_t);
    
    TurnPreCalculation();

    TurnPreCalculation(const TurnPreCalculation&) = default;
    TurnPreCalculation& operator = (const TurnPreCalculation&) = default;

    std::unique_ptr<TurnIterator> generateTurnIterator();
    
    float getTrajEndTime();
    float getPreDist();
    float getFolDist();
  private:
    float _pre_dist;
    float _fol_dist;
    float _shape_factor;
    float _path_length;
    float _target_ang;
    float _start_ang; 
    float _start_x;
    float _start_y;
    float _end_x;
    float _end_y;
    float _int_molli_c;
    float _v;
    float _cp;
    float _delta_t;
    float _end_time;
    float _move_x;
    float _move_y;

    void _init();
    void _init180();
};



