#pragma once

#include <stdint.h>

// Msg
#include "navStateMsg.h"
#include "wallSensorMsg.h"


class Wall {
  public:
    uint8_t E :1; //壁情報
    uint8_t N :1;
    uint8_t W :1;
    uint8_t S :1;
    uint8_t EF :1;//壁を読んだかのフラグ
    uint8_t NF :1;
    uint8_t WF :1;
    uint8_t SF :1;

    void setByUint8(uint8_t wall);
    void print();

    Wall();
    Wall(EAzimuth dir, bool l, bool a, bool r);
};

enum class EUpdateWallStatus : int8_t{
    REACHED = 1,
    UPDATED = 0,
    REACHED_ERROR = -1
};

class Maze {
  public:
    Maze();
    uint32_t walls_vertical[31];
    uint32_t walls_horizontal[31];
    uint32_t reached[32];
    uint32_t no_entry[32];
    uint16_t p_map[32][32];
    uint16_t goal_x;
    uint16_t goal_y;

    void init();
    void initWall();
    void initReached();

    bool isReached(uint8_t x, uint8_t y);
    void writeReached(uint8_t x, uint8_t y, bool reached_);
    bool isCrossroads(uint8_t x, uint8_t y);

    bool isNoEntry(uint8_t x, uint8_t y);
    void writeNoEntry(uint8_t x, uint8_t y, bool no_entry_);
    void writeNoEntryAllFalse();
    void writeNoEntryAllTrue();
    

    Wall readWall(uint8_t x, uint8_t y);
    
    bool existsAWall(uint8_t x, uint8_t y, EAzimuth dir);
    bool existsRWall(uint8_t x, uint8_t y, EAzimuth dir);
    bool existsLWall(uint8_t x, uint8_t y, EAzimuth dir);

    bool watchedRWall(uint8_t x, uint8_t y, EAzimuth dir);
    bool watchedLWall(uint8_t x, uint8_t y, EAzimuth dir);

    bool existsGoalPath(uint8_t x, uint8_t y);

    int8_t calcRotTimes(EAzimuth dest_dir, EAzimuth my_dir);

    void writeAheadWall(uint8_t x, uint8_t y, EAzimuth dir, bool ahead);
    void writeSideWall(uint8_t x, uint8_t y, EAzimuth dir, bool l, bool r);
    void writeWall(uint8_t x, uint8_t y, Wall wall);
    void writeWall(uint8_t x, uint8_t y, EAzimuth dir, bool l, bool a, bool r, bool b=false);
    void writeWall(uint8_t x, uint8_t y, EAzimuth dir, WallSensorMsg& ws_msg);
    void clearAdjacentWall(uint8_t x, uint8_t y);
    
    EUpdateWallStatus updateWall(uint8_t x, uint8_t y, EAzimuth dir, WallSensorMsg& ws_msg);
    EUpdateWallStatus updateWall(uint8_t x, uint8_t y, EAzimuth dir, bool l, bool a, bool r);
    void updateStartSectionWall();

    EAzimuth getMinDirection(uint8_t x, uint8_t y, EAzimuth dir);
    EAzimuth getUnknownDirection(uint8_t x, uint8_t y, EAzimuth dir);

    EAzimuth getSearchDirection(uint8_t x, uint8_t y, EAzimuth dir);    
    
    void makeSearchMap(uint8_t x, uint8_t y);
    void makeAllAreaSearchMap(uint8_t x, uint8_t y);
    void makeRandomFastestMap(uint8_t x, uint8_t y);
    void makeFastestMap(uint8_t x, uint8_t y);
    void makeRandomNoEntryMaskMap(uint8_t x, uint8_t y);

    void watchPotentialMap();

    void serializeMazeData(uint8_t* byte_arr);
    void writeMazeData2Flash();
    void readMazeDataFromFlash();
    uint32_t xor32();    
};


