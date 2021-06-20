#pragma once

#include "BaseMsg.h"

class PositionEstimatorMsg : public BaseMsg{
  public:
    float x = 0.0f;
    float y = 0.0f;
};