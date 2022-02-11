#pragma once

#include "baseMsg.h"
#include "turnEnum.h"

class TrajTripletMsg : public BaseMsg{
  public:
    ETrajType traj_type_pre = ETrajType::NONE; // [enum]
    ETurnType turn_type_pre = ETurnType::NONE; // [enum]
    ETurnDir turn_dir_pre = ETurnDir::NO_TURN; // [enum]
    ETrajType traj_type_now = ETrajType::NONE; // [enum]
    ETurnType turn_type_now = ETurnType::NONE; // [enum]
    ETurnDir turn_dir_now = ETurnDir::NO_TURN; // [enum]
    ETrajType traj_type_next = ETrajType::NONE; // [enum]
    ETurnType turn_type_next = ETurnType::NONE; // [enum]
    ETurnDir turn_dir_next = ETurnDir::NO_TURN; // [enum]

    float end_x_now = 0.0f;                     // [m]
    float end_y_now = 0.0f;                     // [m]
    float end_yaw_now = 0.0f;                   // [rad]
};