
#pragma once

#include <stdint.h>

namespace sim {

    void initSimConnection();
    void finalizeSimConnection();
    void setRobotPos(float x, float y, float ang, float v);
    void setRobotColor(uint8_t r, uint8_t g, uint8_t b);
    void setTargetPos(float x, float y, float ang);
    void setWallsWithoutOuter32(uint32_t* walls_vertical, uint32_t* walls_horizontal);
    void setWallsWithoutOuter32(uint32_t* walls_vertical, uint32_t* walls_horizontal, uint32_t* transparent_v_mask, uint32_t* transparent_h_mask);
    void setNeedle(float x, float y);
    void setReload();
    void setNumberSqure(float num, float x, float y);
    void updateDataView(float x, float y, float ang, float v);
    void addPointRobotContrail(float x, float y, float ang, float v);
    void addPointTargetContrail(float x, float y, float ang, float v);
    void updateDataView(float time, float bat_vol, float l_motor_vol, float r_motor_vol, float x, float y, float yaw, float v);
}