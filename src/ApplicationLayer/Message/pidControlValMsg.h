#pragma once

#include "baseMsg.h"

class PidControlValMsg : public BaseMsg{
  public:   
    float v_p = 0.0f;        // [duty -1 to 1]
    float v_i = 0.0f;        // [duty -1 to 1]
    float v_d = 0.0f;        // [duty -1 to 1]

    float yawrate_p = 0.0f;  // [duty -1 to 1]
    float yawrate_i = 0.0f;  // [dyty -1 to 1]
    float yawrate_d = 0.0f;  // [duty -1 to 1]

    float yaw_p = 0.0f;      // [rad/s]
    float yaw_i = 0.0f;      // [rad/s]
    float yaw_d = 0.0f;      // [rad/s]

    float wall_p = 0.0f;     // [rad/s]
    float wall_i = 0.0f;     // [rad/s]
    float wall_d = 0.0f;     // [rad/s]

    float diag_p = 0.0f;     // [rad/s]
    float diag_i = 0.0f;     // [rad/s]
    float diag_d = 0.0f;     // [rad/s]

    float setp_v_xy_body;    // [m/s]
    float setp_yaw;          // [rad]
    float setp_yawrate;      // [rad/s]
};
