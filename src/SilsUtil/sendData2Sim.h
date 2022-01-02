
#pragma once

#include <stdint.h>

namespace sim {

    void initSimConnection();
    void finalizeSimConnection();
    void setRobotPos(float x, float y, float ang, float v);
    void setRobotColor(uint8_t r, uint8_t g, uint8_t b);
    void setTargetPos(float x, float y, float ang);
    void setMazeWall(uint32_t* walls_vertical, uint32_t* walls_horizontal);
    void setMazeWall(uint32_t* walls_vertical, uint32_t* walls_horizontal, uint32_t* transparent_v_mask, uint32_t* transparent_h_mask);
    void setNeedle(float x, float y);
    void setReload();
    void setNumberSqure(float num, float x, float y);

}