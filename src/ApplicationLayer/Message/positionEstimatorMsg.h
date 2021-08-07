#pragma once

#include "BaseMsg.h"

class PositionEstimatorMsg : public BaseMsg{
  public:
    float x = 0.0f;
    float y = 0.0f;
    float v = 0.0f;
    float v_acc = 0.0f;
    float a = 0.0f;
    
    float beta;
    float ang_x = 0.0f;
    float ang_v = 0.0f;
    float ang_a = 0.0f;

};