#pragma once

#include "baseMsg.h"
#include "stdint.h"

enum class ETrajType : uint8_t{
    STOP = 0,
    STRAIGHT,
    DIAGONAL,    
    SPINTURN,
    CURVE
};

enum class ETurnType : uint8_t{
    STOP = 0,
    STRAIGHT,
    STRAIGHT_WALL_CENTER,    
    DIAGONAL,
    DIAGONAL_CENTER,
    TURN_90,
    TURN_L_90,
    TURN_180,
    TURN_S2D_45,
    TURN_S2D_135,
    TURN_D_90,
    TURN_D2S_45,
    TURN_D2S_135,    
    CIRCULAR    
};

enum class ETurnDir : int8_t{
    CW = -1, 
    NO_TURN = 0, 
    CCW = 1
};


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

    ETrajType traj_type = ETrajType::STOP; // [enum]
    ETurnType turn_type = ETurnType::STOP; // [enum]
    ETurnDir turn_dir = ETurnDir::NO_TURN; // [enum]
};