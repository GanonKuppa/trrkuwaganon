#pragma once

#include "baseMsg.h"

enum class ENavMode {
    STANDBY = 0,
    FASTEST,
    SEARCH,    
};

enum class ENavSubMode {
    STANDBY = 0,
    GOAL2START,
    START2GOAL
};

class NavStateMsg : public BaseMsg {
  public:
    bool armed = false;
    ENavMode mode = 0;
    ENavStateMsg sub_mode = 0;
    uint8_t x_int = 0;
    uint8_t y_int = 0;
    

};