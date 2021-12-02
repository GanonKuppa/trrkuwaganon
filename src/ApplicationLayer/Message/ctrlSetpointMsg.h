#pragma once

#include "baseMsg.h"
#include "turnEnum.h"

class CtrlSetpointMsg : public BaseMsg{
  public:
    float x = 0.0f;         // [m]
    float v_x = 0.0f;       // [m/s]
    float a_x = 0.0f;       // [m/s^2]

    float y = 0.0f;         // [m]
    float v_y = 0.0f;       // [m/s]
    float a_y = 0.0f;       // [m/s^2]

    float v_xy_body = 0.0f; // [m/s]
    float a_xy_body = 0.0f; // [m/s^2]

    float yaw = 0.0f;       // [rad]
    float yawrate = 0.0f;   // [rad/s]
    float yawacc = 0.0f;    // [rad/s^2]

    float beta = 0.0f;      // [rad]
    float beta_dot = 0.0f;  // [rad/s]

    ETrajType traj_type = ETrajType::STOP; // [enum]
    ETurnType turn_type = ETurnType::STOP; // [enum]
    ETurnDir turn_dir = ETurnDir::NO_TURN; // [enum]
};