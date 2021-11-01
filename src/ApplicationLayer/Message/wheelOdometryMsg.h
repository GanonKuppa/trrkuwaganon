#pragma once

#include "baseMsg.h"

class WheelOdometryMsg : public BaseMsg{
  public:
    float v = 0.0;              // [m/s]
    float v_r = 0.0;            // [m/s]
    float v_l = 0.0;            // [m/s]
    float v_r_ave = 0.0f;       // [m/s]
    float v_l_ave = 0.0f;       // [m/s]
    
    float rpm_r = 0.0f;         // [rpm] 
    float rpm_l = 0.0f;         // [rpm]
    float rpm_r_ave = 0.0f;     // [rpm] 
    float rpm_l_ave = 0.0f;     // [rpm]


    float yawrate = 0.0f;       // [rad/s]    

    float ang_r = 0.0f;         // [rad]
    float ang_l = 0.0f;         // [rad]
};