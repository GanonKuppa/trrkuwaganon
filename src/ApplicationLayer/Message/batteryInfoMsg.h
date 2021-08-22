#pragma once

#include "baseMsg.h"

class BatteryInfoMsg : public BaseMsg{
  public:   
    float voltage = 4.2;          // [V]
    float voltage_ave = 4.2;      // [V]
    bool is_low_voltage = false;  // [bool]
};
