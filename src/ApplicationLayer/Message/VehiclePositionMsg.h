#pragma once

#include "baseMsg.h"

class VehiclePositionMsg : public BaseMsg{
  public:      
    float x = 0.0f;                   // [m]
    float y = 0.0f;                   // [m]
    float z = 0.0f;                   // [m]

    float v_xy_body_cmp = 0.0f;       // [m/s]
    float v_xy_body_enc = 0.0f;       // [m/s]
    float v_xy_body_ave = 0.0f;       // [m/s]
    float v_xy_body_acc = 0.0f;       // [m/s]

    float v_x = 0.0f;                 // [m/s]
    float v_y = 0.0f;                 // [m/s]
    float v_z = 0.0f;

    float v_xy_body_for_odom = 0.0f;  // [m/s]
    float v_xy_body_for_ctrl = 0.0f;  // [m/s]
    
    float a_body_x = 0.0f;            // [m/s^2]
    float a_body_y = 0.0f;            // [m/s^2]
    float a_body_z = 0.0f;            // [m/s^2]

    float a_x = 0.0f;                 // [m/s^2]
    float a_y = 0.0f;                 // [m/s^2]
    float a_z = 0.0f;                 // [m/s^2]
};