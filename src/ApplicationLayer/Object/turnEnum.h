#pragma once

#include "stdint.h"
#include <string>

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

inline std::string turnDir2Str(ETurnDir td){    
    switch (td) {
        case ETurnDir::CW:
            return std::string("CW");
        case ETurnDir::NO_TURN:
            return std::string("NO_TURN");
        case ETurnDir::CCW:
            return std::string("CCW");
        default:
            return std::string("");
    }
};

inline std::string trajType2Str(ETrajType tt){    
    switch (tt) {
        case ETrajType::STOP:
            return std::string("STOP");
        case ETrajType::STRAIGHT:
            return std::string("STRAIGHT");
        case ETrajType::DIAGONAL:
            return std::string("DIAGONAL");
        case ETrajType::SPINTURN:
            return std::string("SPINTURN");
        case ETrajType::CURVE:
            return std::string("CURVE");
        default:
            return std::string("");
    }
};

inline std::string turnType2Str(ETurnType tt){    
    switch (tt) {
        case ETurnType::STOP:
            return std::string("STOP");
        case ETurnType::STRAIGHT:
            return std::string("STRAIGHT");
        case ETurnType::STRAIGHT_WALL_CENTER:
            return std::string("STRAIGHT_WALL_CENTER");
        case ETurnType::TURN_90:
            return std::string("TURN_90");
        case ETurnType::TURN_L_90:
            return std::string("TURN_L_90");
        case ETurnType::TURN_180:
            return std::string("TURN_180");
        case ETurnType::TURN_S2D_45:
            return std::string("TURN_S2D_45");
        case ETurnType::TURN_S2D_135:
            return std::string("TURN_135");
        case ETurnType::TURN_D2S_45:
            return std::string("TURN_D2S_45");
        case ETurnType::TURN_D2S_135:
            return std::string("TURN_D2S_135");
        case ETurnType::TURN_D_90:
            return std::string("TURN_D_90");
        case ETurnType::DIAGONAL:
            return std::string("DIAGONAL");
        case ETurnType::DIAGONAL_CENTER:
            return std::string("DIAGONAL_CENTER");
        case ETurnType::CIRCULAR:
            return std::string("CIRCULAR");
        default:
            return std::string("");
    }
};

inline std::string turnParamSet2Str(ETurnParamSet tp){    
    switch (tp) {
        case ETurnParamSet::SEARCH:
            return std::string("SEARCH");
        case ETurnParamSet::SAFE0:
            return std::string("SAFE0");
        case ETurnParamSet::SAFE1:
            return std::string("SAFE1");
        case ETurnParamSet::FAST0:
            return std::string("FAST0");
        case ETurnParamSet::FAST1:
            return std::string("FAST1");
        case ETurnParamSet::FAST2:
            return std::string("FAST2");
        case ETurnParamSet::FAST3:
            return std::string("FAST3");
        case ETurnParamSet::FAST4:
            return std::string("FAST4");
        default:
            return std::string("");
    }
};
