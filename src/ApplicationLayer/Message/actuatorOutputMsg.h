#pragma once

#include "baseMsg.h"

enum class ECtrlMode : uint8_t{
    DIRECT_DUTY_SET = 0,
    PSEUDO_DIAL,
    VEHICLE
};


class ActuatorOutputMsg : public BaseMsg{
  public:
    float duty_r = 0.0f;     // [duty -1 to 1]
    float duty_l = 0.0f;     // [duty -1 to 1]

    float duty_r_v = 0.0f;   // [duty -1 to 1]
    float duty_l_v = 0.0f;   // [duty -1 to 1]

    float duty_r_yaw = 0.0f; // [duty -1 to 1]
    float duty_l_yaw = 0.0f; // [duty -1 to 1]
    
    bool is_error = false;   // [bool]

    ECtrlMode ctrl_mode = ECtrlMode::DIRECT_DUTY_SET; // [enum]

};
