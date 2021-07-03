#pragma once

#include "baseMsg.h"

class ImuMsg : public BaseMsg{
  public:
    float ang_v[3] = {0.0f, 0.0f, 0.0f};
    float acc[3] = {0.0f, 0.0f, 0.0f};
    float temp = 0.0f;
    float stop_time = 0.0f;
    float upsideDown_time = 0.0f;

    bool is_stop = false;
    bool is_upside_down = false;

};