#pragma once

#include "baseMsg.h"

class WallSensorMsg : public BaseMsg{
  public:  
    int16_t ahead_l = 0;
    int16_t ahead_r = 0;
    int16_t left = 0;
    int16_t right = 0;
    
    float ahead_dist_l = 0.0f;
    float ahead_dist_r = 0.0f;
    float ahead_dist = 0.0f;
    float dist_l = 0.0f;
    float dist_r = 0.0f;

    bool is_ahead_l = false;
    bool is_ahead_r = false;
    bool is_left = false;
    bool is_right = false;
    bool is_on_wall_center = false;
    
    bool is_corner_l = false;
    bool is_corner_r = false;

    int16_t center_r = 0;
    int16_t center_l = 0;

    float contract_wall_time = 0.0f;
    float on_wall_ahead_time = 0.0f;
    float on_wall_center_time = 0.0f;
};