#pragma once

#include <map>
#include <stdint.h>
#include <string>

class Intent {
  public:
    Intent() {};
    virtual ~Intent() {};

    std::map<std::string, float> float_param;
    std::map<std::string, double> double_param;
    std::map<std::string, int8_t> int8_t_param;
    std::map<std::string, int16_t> int16_t_param;
    std::map<std::string, int32_t> int32_t_param;
    std::map<std::string, uint8_t> uint8_t_param;
    std::map<std::string, uint16_t> uint16_t_param;
    std::map<std::string, uint32_t> uint32_t_param;
};