#pragma once

#include "baseMsg.h"

class WheelOdometryMsg : public BaseMsg{
  public:
    double v = 0.0;
    double a = 0.0;
    double v_R = 0.0;
    double v_L = 0.0;
    
    double v_ave = 0.0;
    double v_R_ave = 0.0;
    double v_L_ave = 0.0;

    float rpm_R = 0.0f;
    float rpm_L = 0.0f;
    float ang_v = 0.0f;
    float kappa = 0.0f;
    float r = 0.0f;

    float tire_ang_R = 0.0f;
    float tire_ang_L = 0.0f;
};