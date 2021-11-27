#pragma once

#include "turnEnum.h"
#include "debugLog.h"

class Path {
  public:
    ETurnType turn_type;
    uint8_t block_num;
    ETurnDir turn_dir;

    Path() {
        turn_type = ETurnType::STRAIGHT;
        block_num = 0;
        turn_dir =  ETurnDir::NO_TURN;
    }

    Path(ETurnType turn_type_, uint8_t block_num_, ETurnDir turn_dir_) {
        turn_type = turn_type_;
        turn_dir = turn_dir_;
        block_num = block_num_;
    }

    Path(const Path& obj) {
        this->block_num = obj.block_num;
        this->turn_dir = obj.turn_dir;
        this->turn_type = obj.turn_type;
    }

    bool isStraightEnd() {
        if(turn_type == ETurnType::STRAIGHT             ||
           turn_type == ETurnType::STRAIGHT_CENTER      ||
           turn_type == ETurnType::STRAIGHT_CENTER_EDGE ||
           turn_type == ETurnType::TURN_180             ||
           turn_type == ETurnType::TURN_D2S_45          ||
           turn_type == ETurnType::TURN_D2S_135         ||
           turn_type == ETurnType::TURN_L_90            ||
           turn_type == ETurnType::TURN_90       )
        {
            return true;
        } else {
            return false;
        }
    }

    bool isStraightStart() {
        if(turn_type == ETurnType::STRAIGHT             ||
           turn_type == ETurnType::STRAIGHT_CENTER      ||
           turn_type == ETurnType::STRAIGHT_CENTER_EDGE ||
           turn_type == ETurnType::TURN_180             ||
           turn_type == ETurnType::TURN_S2D_45          ||
           turn_type == ETurnType::TURN_S2D_135         ||
           turn_type == ETurnType::TURN_L_90            ||
           turn_type == ETurnType::TURN_90       ) 
        {
            return true;
        } else {
            return false;
        }

    }

    bool isDiagonalEnd() {
        if(turn_type == ETurnType::DIAGONAL             ||
           turn_type == ETurnType::DIAGONAL_CENTER      ||
           turn_type == ETurnType::DIAGONAL_CENTER_EDGE ||
           turn_type == ETurnType::TURN_S2D_45          ||
           turn_type == ETurnType::TURN_S2D_135         ||
           turn_type == ETurnType::TURN_D_90    )
        {
            return true;
        } else {
            return false;
        }
    }

    bool isDiagonalStart() {
        if(turn_type == ETurnType::DIAGONAL             ||
           turn_type == ETurnType::DIAGONAL_CENTER      ||
           turn_type == ETurnType::DIAGONAL_CENTER_EDGE ||           
           turn_type == ETurnType::TURN_D2S_45          ||
           turn_type == ETurnType::TURN_D2S_135         ||
           turn_type == ETurnType::TURN_D_90    )
        {
            return true;
        } else {
            return false;
        }
    }

    ~Path() {

    }


    void print() {
        PRINTF_ASYNC("turn_type %d, block_num %d, turn_dir %d\n", turn_type, block_num, turn_dir);
    }

};

