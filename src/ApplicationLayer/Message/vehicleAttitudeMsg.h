#pragma once

#include "baseMsg.h"

class VehicleAttitudeMsg : public BaseMsg{
  public:      
    float yaw;         // [rad]
    float roll;        // [rad]
    float pitch;       // [rad]

    float q0;          // [quat]
    float q1;          // [quat]
    float q2;          // [quat]
    float q3;          // [quat]

    float roll_acc;    // [rad]
    float pitch_acc;   // [rad]

    float beta;        // [rad]
    float beta_dot;    // [rad]

    float yawrate;     // [rad]
    float rollrate;    // [rad]
    float pitchrate;   // [rad]
};