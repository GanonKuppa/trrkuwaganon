#pragma once

#include "baseMsg.h"
#include <stdint.h>

enum class ENavMode : uint8_t{
    STANDBY = 0,
    FASTEST,
    SEARCH,
    MODE_SELECT,
    DEBUG
};

enum class ENavSubMode : uint8_t{
    STANDBY = 0,
    GOAL2START,
    START2GOAL
};

enum class EAzimuth : uint8_t{
    E = 0, 
    NE = 1, 
    N = 2, 
    NW = 3, 
    W = 4, 
    SW = 5, 
    S = 6, 
    SE = 7,
    POLE = 255
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
    bool r_wall_enable = false;                   // [bool]
    bool l_wall_enable = false;                   // [bool]
    bool in_read_wall_area = false;               // [bool]
    bool is_failsafe = false;                     // [bool]
};
