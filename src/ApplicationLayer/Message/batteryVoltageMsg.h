#pragma once

#include "baseMsg.h"

class BatteryVoltageMsg : public BaseMsg{
  public:   
    float voltage = 4.2;
    float voltage_ave = 4.2;
    bool is_low_voltage = false;
};