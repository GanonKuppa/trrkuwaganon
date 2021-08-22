#pragma once

#include "baseMsg.h"

class DialPositionMsg : public BaseMsg{
  public:
    uint8_t dial_pos_r = 0;
    uint8_t dial_pos_l = 0;
    uint8_t dial_div_num_r = 8;
    uint8_t dial_div_num_l = 8;
};
