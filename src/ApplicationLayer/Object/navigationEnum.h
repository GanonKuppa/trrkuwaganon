#pragma once

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
    START2GOAL,
    START2GOAL2START
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

enum class ENavCommand : uint8_t{
    DO_FIRST_MOVE = 0,
    GO_NEXT_SECTION, 
    GO_FORWARD,
    GO_LEFT,
    GO_RIGHT,
    GO_CENTER,
    DO_UTURN,
    UPDATE_POTENTIAL_MAP                    
};

EAzimuth yaw2Azimuth(float yaw){
    constexpr RAD2DEG = 180.0f / 3.14159265f;
    float yaw_ang = yaw * RAD2DEG;
    if(yaw_ang >= 315.0f || yaw_ang <  45.0f) return EAzimuth::E;
    else if(yaw_ang >=  45.0f && yaw_ang < 135.0f) return = EAzimuth::N;
    else if(yaw_ang >= 135.0f && yaw_ang < 225.0f) return = EAzimuth::W;
    else if(yaw_ang >= 225.0f && yaw_ang < 315.0f) return = EAzimuth::S;
    else EAzimuth::E;
}