#pragma once

#include "baseMsg.h"

class GroundTruthMsg : public BaseMsg{
  public:
    // gyro 
    float gyro_sim[3] = {0.0f, 0.0f, 0.0f};
    
    // acc
    float accel_sim[3] = {0.0f, 0.0f, 0.0f};
    
    // local position
    float v;     // m/s
    float v_x;   // m/s
    float v_y;   // m/s
    float a;
    float a_x;   // m/s^2
    float a_y;   // m/s^2


    // grobal position
    float x;     // m
    float y;     // m
    float x_d;
    float y_d;
    float x_dd;
    float y_dd;

    // attitude
    float ang;   // deg
    float ang_v; // deg/s
    float ang_a; // deg/s^2
    float beta;  // deg

    // encorder
    float v_r;   // m/s
    float v_l;   // ms/
    float rpm_r; // rpm
    float rpm_l; // rpm

    // force
    float f_r;   // N
    float f_l;   // N
    float torque;// N・m
        
    // wall sensor
    int16_t ahead_l;
    int16_t ahead_r;
    int16_t left;
    int16_t right;

    // battery
    float voltage = 4.2; // V
 
};
