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

class Maze {
  public:
    uint32_t walls_vertical[31];
    uint32_t walls_horizontal[31];
    uint32_t reached[32];
    uint32_t no_entry[32];
    uint16_t p_map[32][32];

    void init();
    void initWall();
    void initReached();

    bool isReached(uint16_t x, uint16_t y);
    void writeReached(uint16_t x, uint16_t y, bool reached_);
    
    bool isNoEntry(uint16_t x, uint16_t y);
    void writeNoEntry(uint16_t x, uint16_t y, bool no_entry_);
    void writeNoEntryAllFalse();
    void writeNoEntryAllTrue();
    

    Wall readWall(uint16_t x, uint16_t y);
    
    bool existAWall(uint16_t x, uint16_t y, EAzimuth dir);
    bool existRWall(uint16_t x, uint16_t y, EAzimuth dir);
    bool existLWall(uint16_t x, uint16_t y, EAzimuth dir);

    bool watchedRWall(uint16_t x, uint16_t y, EAzimuth dir);    
    bool watchedLWall(uint16_t x, uint16_t y, EAzimuth dir);

    bool isExistPath(uint16_t x, uint16_t y);

    int8_t calcRotTimes(EAzimuth dest_dir, EAzimuth my_dir);

    void writeAheadWall(uint16_t x, uint16_t y, EAzimuth dir, bool ahead);
    void writeWall(uint16_t x, uint16_t y, Wall wall);
    void writeWall(uint16_t x, uint16_t y, EAzimuth dir, bool l, bool a, bool r);
    void writeWall(uint16_t x, uint16_t y, EAzimuth dir, WallSensorMsg& ws_msg);
    
    int8_t updateWall(uint16_t x, uint16_t y, EAzimuth dir, WallSensorMsg& ws_msg);
    int8_t updateWall(uint16_t x, uint16_t y, EAzimuth dir, bool l, bool a, bool r);
    void updateStartSectionWall();

    EAzimuth getMinDirection(uint16_t x, uint16_t y, EAzimuth dir);
    EAzimuth getUnknownDirection(uint16_t x, uint16_t y, EAzimuth dir);

    EAzimuth getSearchDirection(uint16_t x, uint16_t y, EAzimuth dir);    
    EAzimuth getSearchDirection2(uint16_t x, uint16_t y, EAzimuth dir);
    
    void makeSearchMap(uint16_t x, uint16_t y);
    void makeRandomFastestMap(uint16_t x, uint16_t y);
    void makeFastestMap(uint16_t x, uint16_t y);
    void makeRandomNoEntryMaskMap(uint16_t x, uint16_t y);

    void watchPotentialMap(void);

    void serializeMazeData(uint8_t* byte_arr);
    void writeMazeData2Flash(void);
    void readMazeDataFromFlash(void);
  private:
    uint32_t _xor32(void);
};


