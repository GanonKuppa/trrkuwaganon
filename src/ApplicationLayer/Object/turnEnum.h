#pragma once

#include <stdint.h>
#include <string>

enum class ETrajType : uint8_t{
    STOP = 0,
    SPINTURN,
    STRAIGHT,        
    CURVE,
    AHEAD_WALL_CORRECTION,
    NONE
};

enum class ETurnType : uint8_t{
    STOP = 0,
    SPINTURN = 1,
    STRAIGHT = 2,
    STRAIGHT_CENTER = 3,
    STRAIGHT_CENTER_EDGE = 4,
    DIAGONAL = 5,
    DIAGONAL_EDGE = 6,
    DIAGONAL_CENTER = 7,
    DIAGONAL_CENTER_EDGE = 8,
    TURN_90 = 9,
    TURN_L_90 = 10,
    TURN_180 = 11,
    TURN_S2D_45 = 12,
    TURN_S2D_135 = 13,
    TURN_D_90 = 14,
    TURN_D2S_45 = 15,
    TURN_D2S_135 = 16,    
    CIRCULAR = 17,
    AHEAD_WALL_CORRECTION = 18,
    AHEAD_WALL_YAW_CORRECTION = 19,
    NONE = 20
};

enum class ETurnDir : int8_t{
    CW = -1, 
    NO_TURN = 0, 
    CCW = 1
};

enum class ECornerType : int8_t{
    WALL = 0,
    PILLAR,
    NONE
};

enum class ETurnParamSet : uint8_t{
    SEARCH = 1,
    SAFE,    
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
}

inline std::string trajType2Str(ETrajType tt){    
    switch (tt) {
        case ETrajType::STOP:
            return std::string("STOP");
        case ETrajType::SPINTURN:
            return std::string("SPINTURN");
        case ETrajType::STRAIGHT:
            return std::string("STRAIGHT");
        case ETrajType::CURVE:
            return std::string("CURVE");
        case ETrajType::AHEAD_WALL_CORRECTION:
            return std::string("AHEAD_WALL_CORRECTION");
        case ETrajType::NONE:        
            return std::string("NONE");
        default:
            return std::string("");
    }
}

inline std::string turnType2Str(ETurnType tt){    
    switch (tt) {
        case ETurnType::STOP:
            return std::string("STOP");
        case ETurnType::SPINTURN:
            return std::string("SPINTURN");
        case ETurnType::STRAIGHT:
            return std::string("STRAIGHT");
        case ETurnType::STRAIGHT_CENTER:
            return std::string("STRAIGHT_CENTER");
        case ETurnType::STRAIGHT_CENTER_EDGE:
            return std::string("STRAIGHT_CENTER_EDGE");
        case ETurnType::TURN_90:
            return std::string("TURN_90");
        case ETurnType::TURN_L_90:
            return std::string("TURN_L_90");
        case ETurnType::TURN_180:
            return std::string("TURN_180");
        case ETurnType::TURN_S2D_45:
            return std::string("TURN_S2D_45");
        case ETurnType::TURN_S2D_135:
            return std::string("TURN_S2D_135");
        case ETurnType::TURN_D2S_45:
            return std::string("TURN_D2S_45");
        case ETurnType::TURN_D2S_135:
            return std::string("TURN_D2S_135");
        case ETurnType::TURN_D_90:
            return std::string("TURN_D_90");
        case ETurnType::DIAGONAL:
            return std::string("DIAGONAL");
        case ETurnType::DIAGONAL_EDGE:
            return std::string("DIAGONAL_EDGE");
        case ETurnType::DIAGONAL_CENTER:
            return std::string("DIAGONAL_CENTER");
        case ETurnType::DIAGONAL_CENTER_EDGE:
            return std::string("DIAGONAL_CENTER_EDGE");
        case ETurnType::CIRCULAR:
            return std::string("CIRCULAR");
        case ETurnType::AHEAD_WALL_CORRECTION:
            return std::string("AHEAD_WALL_CORRECTION");
        case ETurnType::NONE:
            return std::string("NONE");
        default:
            return std::string("");
    }
}

inline std::string turnParamSet2Str(ETurnParamSet tp){    
    switch (tp) {
        case ETurnParamSet::SEARCH:
            return std::string("SEARCH");
        case ETurnParamSet::SAFE:
            return std::string("SAFE");
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
}

inline std::string cornerType2Str(ECornerType ct){    
    switch (ct) {
        case ECornerType::PILLAR:
            return std::string("PILLAR");
        case ECornerType::WALL:
            return std::string("WALL");
        case ECornerType::NONE:
            return std::string("NONE");      
        default:
            return std::string("");
    }
}

inline bool isTurnStraight(ETurnType turn_type){
    if(turn_type == ETurnType::STRAIGHT ||
       turn_type == ETurnType::STRAIGHT_CENTER ||
       turn_type == ETurnType::STRAIGHT_CENTER_EDGE ||
       turn_type == ETurnType::DIAGONAL ||
       turn_type == ETurnType::DIAGONAL_EDGE ||
       turn_type == ETurnType::DIAGONAL_CENTER ||
       turn_type == ETurnType::DIAGONAL_CENTER_EDGE
    ){
        return true;
    }
    else {
        return false;        
    }
}

inline bool isTurnCurve(ETurnType turn_type){
    if(turn_type == ETurnType::TURN_90 ||
       turn_type == ETurnType::TURN_L_90 ||
       turn_type == ETurnType::TURN_180 ||
       turn_type == ETurnType::TURN_S2D_45 ||
       turn_type == ETurnType::TURN_S2D_135 ||
       turn_type == ETurnType::TURN_D_90 ||
       turn_type == ETurnType::TURN_D2S_45 ||
       turn_type == ETurnType::TURN_D2S_135
    ){
        return true;
    }
    else {
        return false;        
    }
}
