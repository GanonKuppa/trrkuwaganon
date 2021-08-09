#pragma once

#include "baseMsg.h"

class ImuMsg : public BaseMsg{
  public:   
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;

    
    float pitchrate = 0.0f;
    float rollrate = 0.0f;
    float yawrate = 0.0f;

    float pitchrate_deg = 0.0f;
    float rollrate_deg = 0.0f;
    float yawrate_deg = 0.0f;

    float temp = 0.0f;
    float stop_time = 0.0f;
    float upside_down_time = 0.0f;

    bool is_stop = false;
    bool is_upside_down = false;

};