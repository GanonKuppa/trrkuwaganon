#pragma once

#include "baseMsg.h"

class ImuMsg : public BaseMsg{
  public:   
    float acc_x = 0.0f;            // [m/s^2]
    float acc_y = 0.0f;            // [m/s^2]
    float acc_z = 0.0f;            // [m/s^2]
    
    float pitchrate = 0.0f;        // [rad/s]
    float rollrate = 0.0f;         // [rad/s]
    float yawrate = 0.0f;          // [rad/s]

    float temp = 0.0f;             // [degC]
    float stop_time = 0.0f;        // [s]
    float upside_down_time = 0.0f; // [s]

    bool is_stop = false;          // [bool]
    bool is_upside_down = false;   // [bool]

};