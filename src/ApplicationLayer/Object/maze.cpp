#include "maze.h"

#include <map> // pair
#include <deque>
#include <queue>
#include <algorithm>

// Hal
#include "hal_flashRom.h"

// Lib
#include "debugLog.h"



void Wall::setByUint8(uint8_t wall) {
    E = ((wall >> 0) & 0x01);
    N = ((wall >> 1) & 0x01);
    W = ((wall >> 2) & 0x01);
    S = ((wall >> 3) & 0x01);

    EF = ((wall >> 4) & 0x01);
    NF = ((wall >> 5) & 0x01);
    WF = ((wall >> 6) & 0x01);
    SF = ((wall >> 7) & 0x01);
}

void Wall::print() {
    PRINTF_ASYNC("E:%d, N:%d, W:%d, S:%d\n",E, N, W, S);
}


Wall::Wall() {
    setByUint8(0);
}

Wall::Wall(EAzimuth dir, bool l, bool a, bool r) {
    WF = 1;
    SF = 1;
    EF = 1;
    NF = 1;

    switch(dir) {

        case EAzimuth::E:  //まうすは東向き
            //W = b;
            S = r;
            E = a;
            N = l;
            break;
        case EAzimuth::N://まうすは北向き
            //S = b;
            E = r;
            N = a;
            W = l;
            break;
        case EAzimuth::W://まうすは西向き
            //E = b;
            N = r;
            W = a;
            S = l;
            break;
        case EAzimuth::S://まうすは南向き
            //N = b;
            W = r;
            S = a;
            E = l;
            break;
    }
}


void Maze::init() {
    initWall();
    initReached();
}

void Maze::initWall() {
    for(int x=0; x<32; x++) {
        for(int y=0; y<32; y++) {
            p_map[x][y] = 0;
            Wall wall;
            wall.setByUint8(0);
            writeWall(x,y,wall);
        }
    }
}

void Maze::initReached() {
    for(int i=0; i<32; i++) reached[i] = 0;
}


bool Maze::isReached(uint8_t x, uint8_t y) {
    bool reach;
    if( 0<=x && x<=31 && 0<=y && y<=31) reach =(reached[x] >> y) & 0x00000001;
    else reach = 1;
    return reach;
}

void Maze::writeReached(uint8_t x, uint8_t y, bool reached_) {
    if( 0<=x && x<=31 && 0<=y && y<=31) {
        if(reached_ == true) reached[x] |= (0x01 << y);
        else reached[x] &= reached[x] &= ~(0x01 << y);
    }
}

bool Maze::isNoEntry(uint8_t x, uint8_t y) {
    bool is_no_entry;
    if( 0<=x && x<=31 && 0<=y && y<=31) is_no_entry = (no_entry[x] >> y) & 0x00000001;
    else is_no_entry = 1;
    return is_no_entry;
}

void Maze::writeNoEntry(uint8_t x, uint8_t y, bool no_entry_) {
    if( 0<=x && x<=31 && 0<=y && y<=31) {
        if(no_entry_ == true) no_entry[x] |= (0x01 << y);
        else no_entry[x] &= no_entry[x] &= ~(0x01 << y);
    }
}

void Maze::writeNoEntryAllFalse() {
    for(int i=0; i<32; i++) {
        no_entry[i] = 0;
    }
}

void Maze::writeNoEntryAllTrue() {
    for(int i=0; i<32; i++) {
        no_entry[i] = 0xFFFFFFFF;
    }
}

Wall Maze::readWall(uint8_t x, uint8_t y) {
    Wall wall;
    //壁情報の配列番号に変換
    int8_t v_left = x-1;
    int8_t v_right = x;
    int8_t h_up = y;
    int8_t h_down = y-1;
    //壁番号が範囲外の場合は外周の壁
    wall.E = v_right != 31 ? ((walls_vertical[v_right] >>y )& 1) : 1;
    wall.N = h_up != 31 ? ((walls_horizontal[h_up] >>x )& 1) : 1;
    wall.W = v_left != -1 ? ((walls_vertical[v_left] >>y )& 1) : 1;
    wall.S = h_down != -1 ? ((walls_horizontal[h_down]>>x )& 1) : 1;
    wall.EF = isReached(x, y) || isReached(x+1, y );
    wall.NF = isReached(x, y) || isReached(x, y+1);
    wall.WF = isReached(x, y) || isReached(x-1, y );
    wall.SF = isReached(x, y) || isReached(x, y-1);

    return wall;
};

bool Maze::existAWall(uint8_t x, uint8_t y, EAzimuth dir ) {
    Wall wall = readWall(x, y);
    if(dir == EAzimuth::E) return wall.E && wall.EF;
    else if(dir == EAzimuth::N) return wall.N && wall.NF;
    else if(dir == EAzimuth::W) return wall.W && wall.WF;
    else if(dir == EAzimuth::S) return wall.S && wall.SF;
    else return false;
}

bool Maze::existRWall(uint8_t x, uint8_t y, EAzimuth dir) {
    Wall wall = readWall(x, y);
    if(dir == EAzimuth::E) return wall.S && wall.SF;
    else if(dir == EAzimuth::N) return wall.E && wall.EF;
    else if(dir == EAzimuth::W) return wall.N && wall.NF;
    else if(dir == EAzimuth::S) return wall.W && wall.WF;
    else return false;
}

bool Maze::existLWall(uint8_t x, uint8_t y, EAzimuth dir) {
    Wall wall = readWall(x, y);
    if(dir == EAzimuth::E) return wall.N && wall.NF;
    else if(dir == EAzimuth::N) return wall.W && wall.WF;
    else if(dir == EAzimuth::W) return wall.S && wall.SF;
    else if(dir == EAzimuth::S) return wall.E && wall.EF;
    else return false;
}

bool Maze::watchedRWall(uint8_t x, uint8_t y, EAzimuth dir) {
    Wall wall = readWall(x, y);
    if(dir == EAzimuth::E) return wall.SF;
    else if(dir == EAzimuth::N) return wall.EF;
    else if(dir == EAzimuth::W) return wall.NF;
    else if(dir == EAzimuth::S) return wall.WF;
    else return false;
}

bool Maze::watchedLWall(uint8_t x, uint8_t y, EAzimuth dir) {
    Wall wall = readWall(x, y);
    if(dir == EAzimuth::E) return wall.NF;
    else if(dir == EAzimuth::N) return wall.WF;
    else if(dir == EAzimuth::W) return wall.SF;
    else if(dir == EAzimuth::S) return wall.EF;
    else return false;
}

bool Maze::isExistPath(uint8_t x, uint8_t y) {
    if(p_map[x][y] == 0xffff) return false;
    else return true;
}

int8_t Maze::calcRotTimes(EAzimuth dest_dir, EAzimuth my_dir) {
    int8_t rot_times = (int8_t)dest_dir - (int8_t)my_dir;
    if (rot_times == 6) rot_times = -2;
    else if (rot_times == -6) rot_times = 2;
    else if (rot_times == -4) rot_times = 4;
    return rot_times;
}


void Maze::writeAheadWall(uint8_t x, uint8_t y, EAzimuth dir, bool ahead) {
    PRINTF_ASYNC("  Ahead: %d\n", ahead );
    Wall wall = readWall(x, y);
    if(dir == EAzimuth::E) wall.E = ahead;
    else if(dir == EAzimuth::N) wall.N = ahead;
    else if(dir == EAzimuth::W) wall.W = ahead;
    else if(dir == EAzimuth::S) wall.S = ahead;
    writeWall(x, y, wall);
}

void Maze::writeWall(uint8_t x, uint8_t y, Wall wall) {
    //壁情報の配列番号に変換
    int8_t v_left = x-1;
    int8_t v_right = x;
    int8_t h_up = y;
    int8_t h_down = y-1;
    //壁情報を書き込み
    if(v_right != 31) {
        if(wall.E == 1)walls_vertical[v_right] |= (1 << y);
        else walls_vertical[v_right] &= (~(1 << y));
    }
    if(h_up != 31) {
        if(wall.N == 1)walls_horizontal[h_up] |= (1 << x);
        else walls_horizontal[h_up] &= (~(1 << x));
    }
    if(v_left != -1) {
        if(wall.W == 1)walls_vertical[v_left] |= (1 << y);
        else walls_vertical[v_left] &= (~(1 << y));
    }
    if(h_down != -1) {
        if(wall.S == 1)walls_horizontal[h_down] |= (1 << x);
        else walls_horizontal[h_down] &= (~(1 << x));
    }
}

void Maze::writeWall(uint8_t x, uint8_t y, EAzimuth dir, bool l, bool a, bool r) {
    Wall wall = readWall(x,y);
    switch(dir) {
        case EAzimuth::E:  //まうすは東向き
            wall.W = 0;
            wall.S = r;
            wall.E = a;
            wall.N = l;
            break;
        case EAzimuth::N://まうすは北向き
            wall.S = 0;
            wall.E = r;
            wall.N = a;
            wall.W = l;
            break;
        case EAzimuth::W://まうすは西向き
            wall.E = 0;
            wall.N = r;
            wall.W = a;
            wall.S = l;
            break;
        case EAzimuth::S://まうすは南向き
            wall.N = 0;
            wall.W = r;
            wall.S = a;
            wall.E = l;
            break;
    }
    writeWall(x, y, wall);
}

void Maze::writeWall(uint8_t x, uint8_t y, EAzimuth dir, WallSensorMsg& ws_msg) {
    bool l = ws_msg.is_left;
    bool a = ws_msg.is_ahead;
    bool r = ws_msg.is_right;
    writeWall(x, y, dir, l, a, r);
}

// return 1:  探索済みの区画に来て、迷路情報と現在の壁情報が一致
// return 0:  未探索の区画に現在の壁情報を書き込み
// return -1: 探索済みの区画に来たが、現在の迷路情報と現在の壁情報が矛盾
int8_t Maze::updateWall(uint8_t x, uint8_t y, EAzimuth dir, WallSensorMsg& ws_msg) {
    if(x == 0 && y == 0) {
        writeReached(x, y, true);
        Wall wall;
        wall.E = 1;
        wall.N = 0;
        wall.W = 1;
        writeWall(x, y, wall);
        return 0;
    }

    /////////////探索済み区画に来た時//////////////////
    if(isReached(x,y)) {
        switch(dir) {
            case EAzimuth::E:
                if(readWall(x,y).E != ws_msg.is_ahead ||
                        readWall(x,y).S != ws_msg.is_right ||
                        readWall(x,y).N != ws_msg.is_left
                    ) return -1;
                break;
            case EAzimuth::N:
                if(readWall(x,y).N != ws_msg.is_ahead ||
                        readWall(x,y).E != ws_msg.is_right ||
                        readWall(x,y).W != ws_msg.is_left) return -1;
                break;
            case EAzimuth::W:
                if(readWall(x,y).W != ws_msg.is_ahead||
                        readWall(x,y).N != ws_msg.is_left||
                        readWall(x,y).S != ws_msg.is_right) return -1;
                break;
            case EAzimuth::S:
                if(readWall(x,y).S != ws_msg.is_ahead ||
                        readWall(x,y).W != ws_msg.is_right||
                        readWall(x,y).E != ws_msg.is_left) return -1;
                break;
        }
        return 1;
    }
    ///////////未探索区画に来た時//////////
    else {
        writeReached(x, y, true);
        writeWall(x, y, dir, ws_msg);
        return 0;
    } //end else
}

int8_t Maze::updateWall(uint8_t x, uint8_t y, EAzimuth dir, bool l, bool a, bool r) {
    if(x==0 && y==0) {
        writeReached(x,y,true);
        Wall wall;
        wall.E = 1;
        wall.N = 0;
        wall.W = 1;
        writeWall(x, y, wall);
        return 0;
    }

    writeReached(x,y,true);
    writeWall(x, y, dir, l, a, r);
    return 1;
}

void Maze::updateStartSectionWall() {
    writeReached(0,0,true);
    Wall wall = Wall(EAzimuth::N, true, false, true);
    writeWall(0,0, wall);
}

EAzimuth Maze::getMinDirection(uint8_t x, uint8_t y, EAzimuth dir) {
    uint16_t potential_E = 0xffff;
    uint16_t potential_N = 0xffff;
    uint16_t potential_W = 0xffff;
    uint16_t potential_S = 0xffff;
    EAzimuth min_dir;

    Wall wall = readWall(x,y);
    if(wall.E == 0 && wall.EF == 1 && x != 31)potential_E = p_map[x+1][y];
    if(wall.N == 0 && wall.NF == 1 && y != 31)potential_N = p_map[x][y+1];
    if(wall.W == 0 && wall.WF == 1 && x != 0)potential_W = p_map[x-1][y];
    if(wall.S == 0 && wall.SF == 1 && y != 0)potential_S = p_map[x][y-1];
    uint16_t potential_min = std::min({potential_E, potential_N, potential_W, potential_S});

    //直進有線にしている
    if( (potential_min == potential_E) && (dir == EAzimuth::E) ) min_dir = EAzimuth::E;
    else if( (potential_min == potential_N) && (dir == EAzimuth::N) ) min_dir = EAzimuth::N;
    else if( (potential_min == potential_W) && (dir == EAzimuth::W) ) min_dir = EAzimuth::W;
    else if( (potential_min == potential_S) && (dir == EAzimuth::S) ) min_dir = EAzimuth::S;
    else if(potential_min == potential_E) min_dir = EAzimuth::E;
    else if(potential_min == potential_N) min_dir = EAzimuth::N;
    else if(potential_min == potential_W) min_dir = EAzimuth::W;
    else if(potential_min == potential_S) min_dir = EAzimuth::S;

#ifdef MAZE_DEBUG
    PRINTF_ASYNC("・ %d|",p_map[x][y]);
    readWall(x,y).print();
    PRINTF_ASYNC("→ %d|",p_map[x+1][y]);
    readWall(x + 1,y).print();
    PRINTF_ASYNC("← %d|",p_map[x-1][y]);
    readWall(x -1,y).print();
    PRINTF_ASYNC("↑ %d|",p_map[x][y+1]);
    readWall(x, y + 1).print();
    PRINTF_ASYNC("↓ %d|",p_map[x][y-1]);
    readWall(x,y - 1).print();
    PRINTF_ASYNC("p s%d g%d|",p_map[0][0], p_map[7][7]);
#endif
    return min_dir;
}


EAzimuth Maze::getUnknownDirection(uint8_t x, uint8_t y, EAzimuth dir) {
    uint8_t ran_judge_E = 0;
    uint8_t ran_judge_N = 0;
    uint8_t ran_judge_W = 0;
    uint8_t ran_judge_S = 0;

    Wall wall = readWall(x,y);
    if( (x != 31) && (isReached(x+1,y) == false) && (wall.E == 0) ) {
        ran_judge_E = 1;
    }
    if( (y != 31) && (isReached(x,y+1) == false) && (wall.N == 0) ) {
        ran_judge_N = 1;
    }
    if( (x != 0) && (isReached(x-1,y) == false) && (wall.W == 0) ) {
        ran_judge_W = 1;
    }
    if( (y != 0) && (isReached(x,y-1) == false) && (wall.S == 0)) {
        ran_judge_S = 1;
    }
    
    if(x < 3){
        if(ran_judge_W == 1) return EAzimuth::W;
    }

    if(x > 28){
        if(ran_judge_E == 1) return EAzimuth::E;
    }

    if(y < 3){
        if(ran_judge_S == 1) return EAzimuth::S;
    }

    if(y > 28){
        if(ran_judge_N == 1) return EAzimuth::N;
    }

    if(ran_judge_E == 1) return EAzimuth::E;
    if(ran_judge_N == 1) return EAzimuth::N;
    if(ran_judge_W == 1) return EAzimuth::W;
    if(ran_judge_S == 1) return EAzimuth::S;

    return EAzimuth::POLE;
}


EAzimuth Maze::getSearchDirection(uint8_t x, uint8_t y, EAzimuth dir) {
    EAzimuth min_dir = getMinDirection(x, y, dir);
    EAzimuth unknown_dir = getUnknownDirection(x, y, dir);
    
    if( (x > 3 && x < 28) && (y > 3 && y < 28) ) return min_dir;
    else if(unknown_dir == EAzimuth::POLE) return min_dir;
    else if(unknown_dir != min_dir) return unknown_dir;
    else return (EAzimuth)min_dir;
}

EAzimuth Maze::getSearchDirection2(uint8_t x, uint8_t y, EAzimuth dir) {
    EAzimuth min_dir = getMinDirection(x, y, dir);
    EAzimuth unknown_dir = getUnknownDirection(x, y, dir);

    if(unknown_dir == EAzimuth::POLE) return min_dir;
    else if(unknown_dir != min_dir) return unknown_dir;
    else return min_dir;
}

void Maze::makeSearchMap(uint8_t x, uint8_t y) {
    std::queue<std::pair<uint8_t, uint8_t>> que;            

    //歩数マップの初期化
    for(uint8_t i=0; i<32; i++) {
        for(uint8_t j=0; j<32; j++) {
            p_map[i][j] = 0xffff;
        }
    }
    p_map[x][y] = 0; //目的地のポテンシャルは0

    que.push(std::make_pair(x, y));
    while(!que.empty()) {
        x = que.front().first;
        y = que.front().second;
        que.pop();

        // wall_E
        if((x != 31) && p_map[x+1][y] == 0xffff) {
            if( ((walls_vertical[x] >>y )& 1) == 0){
                p_map[x+1][y] = p_map[x][y] + 1;
                que.push(std::make_pair(x+1,y));
            }
        }
        // wall_N
        if((y != 31) && p_map[x][y+1] == 0xffff) {
            if( ((walls_horizontal[y]  >>x )& 1) == 0){
                p_map[x][y+1] = p_map[x][y] + 1;
                que.push(std::make_pair(x,y+1));
            }
        }
        // wall_W
        if((x !=  0) && p_map[x-1][y] == 0xffff) {
            if( ((walls_vertical[x-1]  >>y )& 1) == 0){
                p_map[x-1][y] = p_map[x][y] + 1;
                que.push(std::make_pair(x-1,y));
            }
        }
        // wall_S
        if((y !=  0) && p_map[x][y-1] == 0xffff) {
            if( ((walls_horizontal[y-1]>>x )& 1) == 0){
                p_map[x][y-1] = p_map[x][y] + 1;
                que.push(std::make_pair(x,y-1));
            }
        }
    }

}

void Maze::makeRandomFastestMap(uint8_t x, uint8_t y) {
    std::deque<std::pair<uint8_t, uint8_t>> que;
    //srand((unsigned int)time(NULL));
    //歩数マップの初期化
    for(uint8_t i=0; i<32; i++) {
        for(uint8_t j=0; j<32; j++) {
            p_map[i][j] = 0xffff;
        }
    }
    p_map[x][y] = 0; //目的地のテンシャルは0

    que.push_back(std::make_pair(x,y));
    while(que.empty() == false) {
        x = que.front().first;
        y = que.front().second;
        Wall wall = readWall(x, y);
        que.pop_front();
        if( (wall.E == 0) && (wall.EF == 1) && (x != 31) && (p_map[x+1][y] == 0xffff) ) {
            p_map[x+1][y] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x+1,y));
        }
        if( (wall.N == 0 ) && (wall.NF == 1 ) && (y != 31) && (p_map[x][y+1] == 0xffff) ) {
            p_map[x][y+1] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x,y+1));
        }
        if( (wall.W == 0) && (wall.WF == 1) && (x != 0) && (p_map[x-1][y] == 0xffff) ) {
            p_map[x-1][y] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x-1,y));
        }
        if( (wall.S == 0) && (wall.SF == 1) && (y != 0) && (p_map[x][y-1] == 0xffff) ) {
            p_map[x][y-1] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x,y-1));
        }
    }

}


void Maze::makeFastestMap(uint8_t x, uint8_t y) {

    std::deque<std::pair<uint8_t, uint8_t>> que;

    //歩数マップの初期化
    for(uint8_t i=0; i<32; i++) {
        for(uint8_t j=0; j<32; j++) {
            p_map[i][j] = 0xffff;
        }
    }
    p_map[x][y] = 0; //目的地のテンシャルは0

    que.push_back(std::make_pair(x,y));
    while(que.empty() == false) {
        x = que.front().first;
        y = que.front().second;
        Wall wall = readWall(x, y);
        que.pop_front();
        if( (wall.E == 0) && (wall.EF == 1) && (x != 31) && (p_map[x+1][y] == 0xffff) ) {
            p_map[x+1][y] = p_map[x][y] + 1;
            que.push_back(std::make_pair(x+1,y));
        }
        if( (wall.N == 0 ) && (wall.NF == 1 ) && (y != 31) && (p_map[x][y+1] == 0xffff) ) {
            p_map[x][y+1] = p_map[x][y] + 1;
            que.push_back(std::make_pair(x,y+1));
        }
        if( (wall.W == 0) && (wall.WF == 1) && (x != 0) && (p_map[x-1][y] == 0xffff) ) {
            p_map[x-1][y] = p_map[x][y] + 1;
            que.push_back(std::make_pair(x-1,y));
        }
        if( (wall.S == 0) && (wall.SF == 1) && (y != 0) && (p_map[x][y-1] == 0xffff) ) {
            p_map[x][y-1] = p_map[x][y] + 1;
            que.push_back(std::make_pair(x,y-1));
        }
    }

}

void Maze::makeRandomNoEntryMaskMap(uint8_t x, uint8_t y) {
    std::deque<std::pair<uint8_t, uint8_t>> que;
    //srand((unsigned int)time(NULL));
    //歩数マップの初期化
    for(uint8_t i=0; i<32; i++) {
        for(uint8_t j=0; j<32; j++) {
            p_map[i][j] = 0xffff;
        }
    }
    p_map[x][y] = 0; //目的地のテンシャルは0

    que.push_back(std::make_pair(x,y));
    while(que.empty() == false) {
        x = que.front().first;
        y = que.front().second;
        Wall wall = readWall((uint8_t)(que.front().first), (uint8_t)(que.front().second));
        que.pop_front();
        if( (wall.E == 0) && !isNoEntry(x+1,y) &&(wall.EF == 1) && (x != 31) && (p_map[x+1][y] == 0xffff) ) {
            p_map[x+1][y] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x+1,y));
        }
        if( (wall.N == 0 ) && !isNoEntry(x,y+1) && (wall.NF == 1 ) && (y != 31) && (p_map[x][y+1] == 0xffff) ) {
            p_map[x][y+1] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x,y+1));
        }
        if( (wall.W == 0) && !isNoEntry(x-1,y) && (wall.WF == 1) && (x != 0) && (p_map[x-1][y] == 0xffff) ) {
            p_map[x-1][y] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x-1,y));
        }
        if( (wall.S == 0) && !isNoEntry(x,y-1) && (wall.SF == 1) && (y != 0) && (p_map[x][y-1] == 0xffff) ) {
            p_map[x][y-1] = p_map[x][y] + 1 + _xor32() % 10;
            que.push_back(std::make_pair(x,y-1));
        }
    }
}

void Maze::watchPotentialMap(void) {
    /////////////////////////////
    for(int j=15; j>=0; j--) {
        for(int i=0; i<16; i++) {
            //////1桁//////////
            if(p_map[i][j]< 10) {
                if(i==0) {
                    PRINTF_ASYNC("%x  %d    ",j, p_map[i][j]);
                } else if(i==15) {
                    PRINTF_ASYNC("%d\n",p_map[i][j]);
                } else {
                    PRINTF_ASYNC("%d    ",p_map[i][j]);
                }
            }
            /////2桁//////////
            if( (p_map[i][j] > 9) && (p_map[i][j] < 99) ) {
                if(i==0) {
                    PRINTF_ASYNC("%x  %d   ",j, p_map[i][j]);
                } else if(i==15) {
                    PRINTF_ASYNC("%d\n",p_map[i][j]);
                } else {
                    PRINTF_ASYNC("%d   ",p_map[i][j]);
                }
            }
            /////3桁//////////
            if(p_map[i][j] > 100) {
                if(i==0) {
                    PRINTF_ASYNC("%x  %d  ",j, p_map[i][j]);
                } else if(i==15) {
                    PRINTF_ASYNC("%d\n",p_map[i][j]);
                } else {
                    PRINTF_ASYNC("%d  ",p_map[i][j]);
                }
            }

        }
    }
    PRINTF_ASYNC("\n   0    1    2    3    4    5    6    7    8    9    a    b    c    d    e    f   \n");
}



    //
void Maze::serializeMazeData(uint8_t* byte_arr) {
    uint8_t* p;

    p = reinterpret_cast<uint8_t*>(&walls_vertical[0]);
    for(uint16_t i=0; i<sizeof(walls_vertical); i++) byte_arr[i] = p[i];

    p = reinterpret_cast<uint8_t*>(&walls_horizontal[0]);
    for(uint16_t i=0; i<sizeof(walls_horizontal); i++) byte_arr[i+sizeof(walls_vertical)] = p[i];

    p = reinterpret_cast<uint8_t*>(&reached[0]);
    for(uint16_t i=0; i<sizeof(reached); i++) byte_arr[i+sizeof(walls_vertical)+sizeof(walls_horizontal)] = p[i];
}

void Maze::writeMazeData2Flash(void) {
    const uint16_t WRITE_TARGET_BLOCK = 512;
    const uint16_t START_INDEX = WRITE_TARGET_BLOCK * hal::FLASH_BLOCK_BYTE_SIZE;
    const uint16_t BLOCK_NUM = 10;
    const uint32_t LEN = hal::FLASH_BLOCK_BYTE_SIZE;

    uint8_t write_val[BLOCK_NUM * LEN];
    serializeMazeData(&write_val[0]);

    //4*31+4*31+4*32 = 376byte
    // データフラッシュを10ブロック分イレイズ
    for(uint8_t i=0; i<BLOCK_NUM; i++) {
        uint32_t index = START_INDEX + i*LEN;
        while(1) {
            if(hal::eraseCheckFlashRom(index, LEN) == false) {
                hal::eraseFlashRom(index);
            };
            hal::writeFlashRom(index, &write_val[i*LEN], LEN);
            uint8_t read_val[LEN];
            hal::readFlashRom(index, &read_val[0], LEN);

            bool check_result = true;
            for(uint8_t j=0; j<LEN; j++) {
                if(read_val[j] != write_val[i*LEN+j]) {
                    PRINTF_ASYNC("write error!\n");
                    check_result = false;
                }
            }
            if(check_result == true) break;

        }
    }
}

void Maze::readMazeDataFromFlash(void) {
    const uint16_t WRITE_TARGET_BLOCK = 512;
    const uint16_t START_INDEX = WRITE_TARGET_BLOCK * hal::FLASH_BLOCK_BYTE_SIZE;
    const uint16_t BLOCK_NUM = 10;
    const uint32_t LEN = hal::FLASH_BLOCK_BYTE_SIZE * BLOCK_NUM;


    uint8_t byte_arr[LEN];

    uint8_t* p;
    hal::readFlashRom(START_INDEX, &byte_arr[0], LEN);

    p = reinterpret_cast<uint8_t*>(&walls_vertical[0]);
    for(uint16_t i=0; i<sizeof(walls_vertical); i++) p[i] = byte_arr[i];

    p = reinterpret_cast<uint8_t*>(&walls_horizontal[0]);
    for(uint16_t i=0; i<sizeof(walls_horizontal); i++) p[i] = byte_arr[i+sizeof(walls_vertical)];

    p = reinterpret_cast<uint8_t*>(&reached[0]);
    for(uint16_t i=0; i<sizeof(reached); i++) p[i] = byte_arr[i+sizeof(walls_vertical) +sizeof(walls_horizontal) ];

}

uint32_t Maze::_xor32(void) {
    static uint32_t y = 2463534242;
    y = y ^ (y << 13);
    y = y ^ (y >> 17);
    return y = y ^ (y << 5);
}
