#pragma once

#include "baseMsg.h"

enum class ECtrlMode {
    DIRECT_DUTY_SET = 0,
    PSEUDO_DIAL,
    VEHICLE
};


class ActuatorOutputMsg : public BaseMsg{
  public:
    float duty_r = 0.0f;     // [duty -1 to 1]
    float duty_l = 0.0f;     // [duty -1 to 1]
    
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
    
    ECtrlMode ctrl_mode = ECtrlMode::DIRECT_DUTY_SET; // [enum]

};
