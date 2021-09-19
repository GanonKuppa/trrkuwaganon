#pragma once

#include "stdint.h"

enum class ETrajType : uint8_t{
    STOP = 0,
    STRAIGHT,
    DIAGONAL,    
    SPINTURN,
    CURVE
};

enum class ETurnType : uint8_t{
    STOP = 0,
    STRAIGHT,
    STRAIGHT_WALL_CENTER,    
    DIAGONAL,
    DIAGONAL_CENTER,
    TURN_90,
    TURN_L_90,
    TURN_180,
    TURN_S2D_45,
    TURN_S2D_135,
    TURN_D_90,
    TURN_D2S_45,
    TURN_D2S_135,    
    CIRCULAR
};

enum class ETurnDir : int8_t{
    CW = -1, 
    NO_TURN = 0, 
    CCW = 1
};

enum class ETurnParamSet : uint8_t{
    SEARCH = 0,
    SAFE0,
    SAFE1,
    FAST0,
    FAST1,
    FAST2,
    FAST3,
    FAST4
};
