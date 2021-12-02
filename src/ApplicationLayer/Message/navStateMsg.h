#pragma once

#include "baseMsg.h"
#include <stdint.h>
#include "navigationEnum.h"


class NavStateMsg : public BaseMsg {
  public:    
    bool navigating = false;                      // [bool]
    ENavMode mode = ENavMode::STANDBY;            // [enum]
    ENavSubMode sub_mode = ENavSubMode::STANDBY;  // [enum]    
    uint8_t x_cur = 0;                            // [coor]
    uint8_t y_cur = 0;                            // [coor]

    EAzimuth azimuth = EAzimuth::N;               // [enum]
    bool r_wall_enable = false;                   // [bool]
    bool l_wall_enable = false;                   // [bool]
    bool in_read_wall_area = false;               // [bool]
    bool is_failsafe = false;                     // [bool]
};
