#pragma once

#include "baseMsg.h"

enum class ENavMode {
    STANDBY = 0,
    FASTEST,
    SEARCH,
    MODE_SELECT
};

enum class ENavSubMode {
    STANDBY = 0,
    GOAL2START,
    START2GOAL
};

enum class EAzimuth {
    E = 0, 
    NE, 
    N, 
    NW, 
    W, 
    SW, 
    S, 
    SE
};


class NavStateMsg : public BaseMsg {
  public:
    bool armed = false;                           // [bool]
    ENavMode mode = ENavMode::STANDBY;            // [enum]
    ENavSubMode sub_mode = ENavSubMode::STANDBY;  // [enum]
    int8_t x_cur = 0;                             // [coor]
    int8_t y_cur = 0;                             // [coor]

    int8_t x_next = 0;                            // [coor]
    int8_t y_next = 0;                            // [coor]

    EAzimuth azimuth = EAzimuth::N;               // [enum]

    bool is_failsage = false;                     // [bool]
};
