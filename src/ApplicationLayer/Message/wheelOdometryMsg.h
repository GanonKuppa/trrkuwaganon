#pragma once

#include "baseMsg.h"

class WheelOdometryMsg : public BaseMsg{
  public:
    float v = 0.0;              // [m/s]
    float v_r = 0.0;            // [m/s]
    float v_l = 0.0;            // [m/s]
    
    float rpm_r = 0.0f;         // [rpm] 
    float rpm_l = 0.0f;         // [rpm]
    
    float ang_v = 0.0f;         // [deg/s]
    float ang_v_rad = 0.0f;     // [rad/s]

    float ang_r = 0.0f;         // [deg]
    float ang_l = 0.0f;         // [deg]
};