#pragma once

#include "baseMsg.h"

class CtrlSetpointMsg : public BaseMsg{
  public:
    uint8_t motion_type = 0;
    uint8_t turn_type = 0;

    float x = 0.0f;
    float x_d = 0.0f;
    float x_dd = 0.0f;
    float y = 0.0f;
    float y_d = 0.0f;
    float y_dd = 0.0f;
    float ang = 0.0f;
    float ang_v = 0.0f;
    float ang_a = 0.0f;
    float v = 0.0f;
    float a = 0.0f;
};