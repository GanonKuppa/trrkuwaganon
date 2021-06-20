#pragma once

#include "baseMsg.h"

class GamepadMsg : public BaseMsg{
  public:
    bool connected = false;
    int8_t cross_x = 0;
    int8_t cross_y = 0;
    int8_t L3D_x = 0;
    int8_t L3D_y = 0;
    int8_t R3D_x = 0;
    int8_t R3D_y = 0;
    uint8_t RT = 0;
    uint8_t LT = 0;
    uint32_t A = 0;
    uint32_t B = 0;
    uint32_t Y = 0;
    uint32_t X = 0;
    uint32_t RB = 0;
    uint32_t LB = 0;
    uint32_t BACK = 0;
    uint32_t START = 0;
};