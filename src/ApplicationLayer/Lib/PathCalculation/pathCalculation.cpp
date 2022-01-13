#include "pathCalculation.h"

#include "stdint.h"
#include <map> // pair
#include <vector>
#include <float.h>
#include <algorithm>
#include <cmath>

// Hal
#include "hal_timer.h"
#include "hal_critical_section.h"

// Lib
#include "debugLog.h"
#include "pathCompression.h"
#include "path.h"

// Module
#include "parameterManager.h"
#include "trajectoryInitializer.h"

// Obj
#include "trajectoryFactory.h"
#include "maze.h"
#include "turnEnum.h"






void setEntryArea(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, Maze& maze) {
    EAzimuth dir = EAzimuth::N;
    uint16_t p_x = start_x;
    uint16_t p_y = start_y;

    while ((p_x != goal_x) || (p_y != goal_y)) {
        //Sleep(20);
        //sendNumberSqure(maze.p_map[p_x][p_y],p_x,p_y);
        maze.writeNoEntry(p_x, p_y, false);
        EAzimuth min_dir = maze.getMinDirection(p_x, p_y, dir);
        dir = min_dir;
        if (dir == EAzimuth::E) p_x++;
        else if (dir == EAzimuth::N) p_y++;
        else if (dir == EAzimuth::W) p_x--;
        else if (dir == EAzimuth::S) p_y--;
    }
}

void makeMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec) {

    EAzimuth dir = EAzimuth::N;
    uint16_t p_x = 0;
    uint16_t p_y = 0;

    maze.makeFastestMap(goal_x, goal_y);
    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
    p_y++;

    while ((p_x != goal_x) || (p_y != goal_y)) {
        //waitmsec(3000);
        EAzimuth min_dir = maze.getMinDirection(p_x, p_y, dir);
        if (min_dir == dir) {
            path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
            path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
        } else {
            int8_t dir_diff = (int8_t)min_dir - (int8_t)dir;
            if (dir_diff == 6){
                dir_diff = -2;
            }                
            if (dir_diff == -6){
                dir_diff = 2;
            }
            int8_t sign = 0;
            if(dir_diff > 0) sign = 1;
            else if(dir_diff < 0) sign = -1;
            else sign = 0;
            
            ETurnDir turn_dir = (ETurnDir)(sign);
            path_vec.push_back(Path(ETurnType::TURN_90, 1, turn_dir));
        }

        dir = min_dir;
        if (dir == EAzimuth::E) p_x++;
        else if (dir == EAzimuth::N) p_y++;
        else if (dir == EAzimuth::W) p_x--;
        else if (dir == EAzimuth::S) p_y--;
#ifdef SIM_TEST
        Sleep(20);
        PRINTF_ASYNC("%d %d %d\n", p_x, p_y, dir);
        sendRobotPos(p_x *0.09 +0.045, p_y*0.09 +0.045, dir * 45.0, 0.0);
#endif
    }
    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
}

void makeQuasiMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec) {
    uint16_t start_x = 0;
    uint16_t start_y = 0;
    uint16_t tmp_goal_x;
    uint16_t tmp_goal_y;
    maze.writeNoEntryAllTrue();

    while(1) {
        tmp_goal_x= maze.xor32() % 32;
        tmp_goal_y= maze.xor32() % 32;
        maze.makeRandomFastestMap(tmp_goal_x, tmp_goal_y);
        if(tmp_goal_x != goal_x && tmp_goal_y != goal_y && maze.p_map[0][0] != 65535) break;
    }

    //PRINTF_ASYNC("tmp_goal:%d %d %d\n", tmp_goal_x, tmp_goal_y, maze.p_map[0][0]);
    setEntryArea(start_x, start_y, tmp_goal_x, tmp_goal_y, maze);

    maze.makeRandomFastestMap(goal_x, goal_y);
    setEntryArea(tmp_goal_x, tmp_goal_y, goal_x, goal_y, maze);

    EAzimuth dir = EAzimuth::N;
    uint16_t p_x = start_x;
    uint16_t p_y = start_y;

    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
    //sendNumberSqure(maze.p_map[p_x][p_y],p_x,p_y);
    p_y++;

    maze.makeRandomNoEntryMaskMap(goal_x, goal_y);
    while ((p_x != goal_x) || (p_y != goal_y)) {
        //sendNumberSqure(maze.p_map[p_x][p_y],p_x,p_y);
        EAzimuth min_dir = maze.getMinDirection(p_x, p_y, dir);
        if (min_dir == dir) {
            path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
            path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
        } else {
            int8_t dir_diff = (int8_t)min_dir - (int8_t)dir;
            if (dir_diff == 6) dir_diff = -2;
            else if (dir_diff == -6) dir_diff = 2;

            int8_t sign = 0;
            if(dir_diff > 0) sign = 1;
            else if(dir_diff < 0) sign = -1;
            else sign = 0;


            ETurnDir turn_dir = (ETurnDir)(sign);
            path_vec.push_back(Path(ETurnType::TURN_90, 1, turn_dir));
        }

        dir = min_dir;
        if (dir == EAzimuth::E) p_x++;
        else if (dir == EAzimuth::N) p_y++;
        else if (dir == EAzimuth::W) p_x--;
        else if (dir == EAzimuth::S) p_y--;

        //Sleep(20);
        //PRINTF_ASYNC("%d %d %d\n", p_x, p_y, dir);
        //sendRobotPos(p_x *0.09 +0.045, p_y*0.09 +0.045 , dir * 45.0, 0.0);
    }
    //sendNumberSqure(0,p_x,p_y);
    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));
}

uint32_t calcPathVecHash(std::vector<Path>& path_vec) {
    uint32_t ret = 0;
    int j = 0;
    for (auto i : path_vec) {
        ret += (uint8_t(i.block_num) + uint8_t(i.turn_type)*(3<<j) + uint8_t(i.turn_dir)*(5<<j));
        if(j>16)j = 0;
        j++;
    }
    return ret;
}

void makeFastestDiagonalPath(uint32_t trial_times, ETurnParamSet tp, uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec) {
    hal::enterCriticalSection();
    
    float min_time = FLT_MAX;
    std::vector<Path> min_vec;
    std::vector<uint32_t> path_vec_hash_list;
    for(uint32_t i=0; i<trial_times; i++) {
        path_vec.clear();
        makeQuasiMinStepPath(goal_x, goal_y, maze, path_vec);            
        uint32_t path_vec_hash = calcPathVecHash(path_vec);
        PRINTF_ASYNC("%d:hash num: %x | vector capacity: %d\n", i, path_vec_hash, path_vec.capacity());
        if ( std::find(path_vec_hash_list.begin(), path_vec_hash_list.end(), path_vec_hash) != path_vec_hash_list.end() ) {
            // do nothing
        } else {
            path_vec_hash_list.push_back(path_vec_hash);
            translatePathDiagonal(path_vec);
            float necessary_time = HF_calcPlayPathTime(tp, path_vec);
            if(min_time > necessary_time) {
                min_time = necessary_time;
                min_vec = path_vec;
            }
        }
        //printf("necessary_time:%f\n", necessary_time);
        //resetWdt();
    }
    path_vec = min_vec;
    PRINTF_ASYNC("path_vec_hash_num:%d\n", path_vec_hash_list.size());
    PRINTF_ASYNC("min_time:%f\n", min_time);

    hal::leaveCriticalSection();

};


void translatePathSpin(std::vector<Path>& path_vec) {
    compress_straight(path_vec);
}


void translatePath90Deg(std::vector<Path>& path_vec) {
    path_vec.insert(path_vec.begin(), Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));

    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN)); // パスの最後に直線がないと変換がうまくできないのでダミーで入れた
    path_vec.pop_back(); // ダミーを消去

    compress_straight(path_vec, 1);
}


void translatePathLong(std::vector<Path>& path_vec) {
    path_vec.insert(path_vec.begin(), Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));

    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN)); // パスの最後に直線がないと変換がうまくできないのでダミーで入れた
    compress_l_90(path_vec);
    compress_180(path_vec);
    path_vec.pop_back(); // ダミーを消去

    compress_straight(path_vec, 1);
}

void translatePathDiagonal(std::vector<Path>& path_vec) {
    path_vec.insert(path_vec.begin(), Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN));

    path_vec.push_back(Path(ETurnType::STRAIGHT, 1, ETurnDir::NO_TURN)); // パスの最後に直線がないと変換がうまくできないのでダミーで入れた
    compress_l_90(path_vec);
    compress_180(path_vec);
    compress_s2d_135(path_vec);
    compress_d2s_135(path_vec);
    compress_s2d_45(path_vec);
    compress_d2s_45(path_vec);
    compress_d_90(path_vec);

    path_vec.pop_back(); // ダミーを消去
    compress_straight(path_vec, 1);
    compress_d_straight(path_vec);
}


void HF_playPath(ETurnParamSet tp, std::vector<Path>& path_vec) {
    TurnParameter turn_p = module::TrajectoryInitializer::getInstance().getTurnParameter(tp);
    float wall2mouse_center_dist = module::ParameterManager::getInstance().wall2mouse_center_dist;
    float v_pre = 0.0f;
    float v_fol = 0.0f;
    bool is_start_section = true;

    float x, v, a, v_max;
    ETurnDir dir;

    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++) {
        if(path_vec[i].turn_type == ETurnType::STRAIGHT) {
            if (is_start_section && path_vec[i+1].turn_type == ETurnType::STRAIGHT ) {
                x = wall2mouse_center_dist + float(path_vec[i+1].block_num) * 0.045f;
                path_vec.erase(path_vec.begin() + i + 1);
            } else if (is_start_section && path_vec[i+1].turn_type != ETurnType::STRAIGHT) {
                x = wall2mouse_center_dist;
            } else {
                x = float(path_vec[i].block_num) * 0.045f;
            }
            is_start_section = false;

            a = turn_p.getAcc(ETurnType::STRAIGHT);
            v_max = turn_p.getV(ETurnType::STRAIGHT);

            if (i == 0) v_pre = 0.0f;
            else v_pre = turn_p.getV(path_vec[i-1].turn_type);

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.0f;
            else v_fol = turn_p.getV(path_vec[i+1].turn_type);
            
            StraightFactory::push(ETurnType::STRAIGHT_CENTER, x, v_pre, v_max, v_fol, a, a);
            

        } else if(path_vec[i].turn_type == ETurnType::DIAGONAL) {
        	a = turn_p.getAcc(ETurnType::DIAGONAL);
            v_max = turn_p.getV(ETurnType::DIAGONAL);
            x = float(path_vec[i].block_num) * 0.045f * 1.4142356f;
            v_pre = turn_p.getV(path_vec[i-1].turn_type);

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.0f;
            else v_fol = turn_p.getV(path_vec[i+1].turn_type);
            
            StraightFactory::push(ETurnType::DIAGONAL, x, v_pre, v_max, v_fol, a, a);            

        } else {
            v = turn_p.getV(path_vec[i].turn_type);
            dir = path_vec[i].turn_dir;
            
            if (i == 0){
                v_pre = 0.0f;
            }
            else{
                if(path_vec[i-1].turn_type != ETurnType::STRAIGHT){
                    v_pre = turn_p.getV(path_vec[i-1].turn_type);
                }
                else{
                    v_pre = turn_p.getV(path_vec[i].turn_type);
                }
            }
            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.0f;
            else v_fol = turn_p.getV(path_vec[i+1].turn_type);
            a = turn_p.getAcc(ETurnType::STRAIGHT);
            float pre_dist = module::TrajectoryInitializer::getInstance().getPreDist(tp ,path_vec[i].turn_type);
            float fol_dist = module::TrajectoryInitializer::getInstance().getFolDist(tp ,path_vec[i].turn_type);
            float a_pre = std::max(a, std::fabs(v_pre * v_pre -  v * v) / (2.0f * pre_dist));
            float a_fol = std::max(a, std::fabs(v_fol * v_fol -  v * v) / (2.0f * fol_dist));

            if(path_vec[i].isStraightStart()) {
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, pre_dist, v_pre, v, v, a_pre, a_pre); 
            } else {
                StraightFactory::push(ETurnType::DIAGONAL, pre_dist, v_pre, v, v, a_pre, a_pre); 
            }
            CurveFactory::push(tp, path_vec[i].turn_type, dir);
            if(path_vec[i].isStraightEnd()) {
                StraightFactory::push(ETurnType::STRAIGHT_CENTER, fol_dist, v, v, v_fol, a_fol, a_fol);
            } else {
                StraightFactory::push(ETurnType::DIAGONAL, fol_dist, v, v, v_fol, a_fol, a_fol);
            }
            

        }
    }
    StopFactory::push(2.0f);
}

float HF_calcPlayPathTime(ETurnParamSet tp, std::vector<Path>& path_vec) {
    float necessary_time = 0.0;

    TurnParameter turn_p = module::TrajectoryInitializer::getInstance().getTurnParameter(tp);
    float wall2mouse_center_dist = module::ParameterManager::getInstance().wall2mouse_center_dist;
    float v_pre = 0.0f;
    float v_fol = 0.0f;
    bool is_start_section = true;

    float x, v, a, v_max;
    ETurnDir dir;

    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++) {
        // 時間を算出する際にpath_vec.eraseを行うとpath_vecの内容が変わってしまい、後にplay_pathするのに支障が生じるためeraseはコメントアウト
        if(i == 1 && path_vec[i].turn_type == ETurnType::STRAIGHT) continue;
        
        if(path_vec[i].turn_type == ETurnType::STRAIGHT) {
            if (is_start_section && path_vec[i+1].turn_type == ETurnType::STRAIGHT ) {
                x = wall2mouse_center_dist + float(path_vec[i+1].block_num) * 0.045f;
                //path_vec.erase(path_vec.begin() + i + 1);
            } else if (is_start_section && path_vec[i+1].turn_type != ETurnType::STRAIGHT) {
                x = wall2mouse_center_dist;
            } else {
                x = float(path_vec[i].block_num) * 0.045f;
            }
            is_start_section = false;

            a = turn_p.getAcc(ETurnType::STRAIGHT);
            v_max = turn_p.getV(ETurnType::STRAIGHT);

            if (i == 0) v_pre = 0.0f;
            else v_pre = turn_p.getV(path_vec[i-1].turn_type);

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.0f;
            else v_fol = turn_p.getV(path_vec[i+1].turn_type);
            
            necessary_time += StraightFactory::create(ETurnType::STRAIGHT_CENTER, x, v_pre, v_max, v_fol, a, a)->getNecessaryTime();
            

        } else if(path_vec[i].turn_type == ETurnType::DIAGONAL) {
            a = turn_p.getAcc(ETurnType::DIAGONAL);
            v_max = turn_p.getV(ETurnType::DIAGONAL);
            x = float(path_vec[i].block_num) * 0.045f * 1.4142356f;
            v_pre = turn_p.getV(path_vec[i-1].turn_type);

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.0f;
            else v_fol = turn_p.getV(path_vec[i+1].turn_type);
            
            necessary_time += StraightFactory::create(ETurnType::DIAGONAL, x, v_pre, v_max, v_fol, a, a)->getNecessaryTime();

        } else {
            v = turn_p.getV(path_vec[i].turn_type);
            dir = path_vec[i].turn_dir;
            {

                if (i == 0){
                    v_pre = 0.0f;
                }
                else{
                    if(path_vec[i-1].turn_type != ETurnType::STRAIGHT){
                        v_pre = turn_p.getV(path_vec[i-1].turn_type);
                    }
                    else{
                        v_pre = turn_p.getV(path_vec[i].turn_type);
                    }
                }

                if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.0f;
                else v_fol = turn_p.getV(path_vec[i+1].turn_type);
                a = turn_p.getAcc(ETurnType::STRAIGHT);
                float pre_dist = module::TrajectoryInitializer::getInstance().getPreDist(tp, path_vec[i].turn_type);
                float fol_dist = module::TrajectoryInitializer::getInstance().getFolDist(tp, path_vec[i].turn_type);
                float a_pre = std::max(a, std::fabs(v_pre * v_pre -  v * v) / (2.0f * pre_dist));
                float a_fol = std::max(a, std::fabs(v_fol * v_fol -  v * v) / (2.0f * fol_dist));

                if(path_vec[i].isStraightStart()) {
                    necessary_time += StraightFactory::create(ETurnType::STRAIGHT_CENTER, pre_dist, v_pre, v, v, a_pre, a_pre)->getNecessaryTime();
                } else {
                    necessary_time += StraightFactory::create(ETurnType::DIAGONAL, pre_dist, v_pre, v, v, a_pre, a_pre)->getNecessaryTime();
                }
                necessary_time += CurveFactory::create(tp, path_vec[i].turn_type, dir)->getNecessaryTime();
                if(path_vec[i].isStraightEnd()) {
                    necessary_time += StraightFactory::create(ETurnType::STRAIGHT_CENTER, fol_dist, v, v, v_fol, a_fol, a_fol)->getNecessaryTime();
                } else {
                    necessary_time += StraightFactory::create(ETurnType::DIAGONAL, fol_dist, v, v, v_fol, a_fol, a_fol)->getNecessaryTime();
                }
            }

        }
    }

    return necessary_time;
}


/*
void HF_playPathSpin(TurnParameter turn_p, std::vector<Path>& path_vec, TrajectoryCommander& trajCommander) {
    UMouse& m = UMouse::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;
    bool is_start_section = true;
    ParameterManager& pm = module::ParameterManager::getInstance();

    float x, v, a, v_max, dir;
    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++) {
        v_pre = v_fol;
        switch (path_vec[i].turn_type) {
            case ETurnType::STRAIGHT:
                x = float(path_vec[i].block_num) * 0.045;
                if (is_start_section == true)
                    x += m.WALL2MOUSE_CENTER_DIST;

                is_start_section = false;
                a = turn_p.a_straight;
                v_max = turn_p.getTurnV(ETurnType::STRAIGHT);

                if (i + 1 == (uint16_t)path_vec.size())
                    v_fol = 0.05;
                else
                    v_fol = turn_p.getTurnV((ETurnType)path_vec[i + 1].turn_type);

                {
                    auto traj_straight = StraightTrajectory::createAsWallCenter(x, v_pre, v_max, v_fol, a, a);
                    trajCommander.push(std::move(traj_straight));
                }
                break;
            case ETurnType::TURN_90:
                v = turn_p.v_turn_90;
                a = turn_p.a_straight;
                dir = SIGN((float)path_vec[i].turn_dir);
                {
                    auto traj0 = StraightTrajectory::create(0.03, v, v, 0.05, a, a);
                    auto traj1 = StraightTrajectory::create(0.015, 0.05, 0.05, 0.05, a, a);

                    auto traj2 = StopTrajectory::create(0.15);
                    auto traj3 = SpinTurnTrajectory::create(dir * 90.0f, pm.spin_ang_v, pm.spin_ang_a);
                    auto traj4 = StopTrajectory::create(0.15);
                    auto traj5 = StraightTrajectory::create(0.03, 0.0, v, v, a, a);
                    auto traj6 = StraightTrajectory::create(0.015, v, v, v, a, a);

                    trajCommander.push(std::move(traj0));
                    trajCommander.push(std::move(traj1));
                    trajCommander.push(std::move(traj2));
                    trajCommander.push(std::move(traj3));
                    trajCommander.push(std::move(traj4));
                    trajCommander.push(std::move(traj5));
                    trajCommander.push(std::move(traj6));
                }
                break;
        }
    }
}


void HF_playPathSpinDiagonal(TurnParameter turn_p, std::vector<Path>& path_vec, TrajectoryCommander& trajCommander) {
    UMouse& m = UMouse::getInstance();
    float v_pre = 0.0;
    float v_fol = 0.0;
    bool is_start_section = true;

    float x, v, a, v_max, dir;


    for (uint16_t i = 0; i < (uint16_t)path_vec.size(); i++) {
        if(path_vec[i].turn_type == ETurnType::STRAIGHT) {
            if (is_start_section == true && path_vec[i+1].turn_type == ETurnType::STRAIGHT ) {
                x = m.WALL2MOUSE_CENTER_DIST + float(path_vec[i+1].block_num) * 0.045;
                path_vec.erase(path_vec.begin() + i + 1);
            } else if (is_start_section == true && path_vec[i+1].turn_type != ETurnType::STRAIGHT) {
                x = m.WALL2MOUSE_CENTER_DIST;
            } else {
                x = float(path_vec[i].block_num) * 0.045;
            }
            is_start_section = false;

            a = turn_p.a_straight;
            v_max = turn_p.getTurnV(ETurnType::STRAIGHT);

            if (i == 0) v_pre = 0.0;
            else v_pre = turn_p.v_turn_90;

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.05;
            else v_fol = turn_p.v_turn_90;

            {
                if(path_vec[i].block_num != 1) {
                    auto traj_straight0 = StraightTrajectory::createAsWallCenter(x - 0.045, v_pre, v_max, v_fol, a, a);
                    auto traj_straight1 = StraightTrajectory::createAsWallCenter(0.045, v_fol, v_fol, v_fol, a, a);
                    trajCommander.push(std::move(traj_straight0));
                    trajCommander.push(std::move(traj_straight1));
                } else {
                    auto traj_straight0 = StraightTrajectory::createAsWallCenter(x, v_pre, v_max, v_fol, a, a);
                    trajCommander.push(std::move(traj_straight0));
                }
            }

        } else if(path_vec[i].turn_type == ETurnType::D_STRAIGHT) {
            a = turn_p.a_d_straight;
            v_max = turn_p.getTurnV(ETurnType::D_STRAIGHT);
            x = float(path_vec[i].block_num) * 0.045 * 1.4142356;
            v_pre = turn_p.v_turn_90;

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.05;
            else v_fol = turn_p.v_turn_90;

            {
                auto traj_straight0 = StraightTrajectory::createAsDiagonal(x - 0.045, v_pre, v_max, v_fol, a, a);
                auto traj_straight1 = StraightTrajectory::createAsDiagonalCenter(0.045, v_fol, v_fol, v_fol, a, a);
                trajCommander.push(std::move(traj_straight0));
                trajCommander.push(std::move(traj_straight1));
            }

        } else {
            v = turn_p.getTurnV(path_vec[i].turn_type);
            dir = (float)path_vec[i].turn_dir;
            if (i == 0) v_pre = 0.0;
            else v_pre = turn_p.v_turn_90;

            if (i + 1 == (uint16_t)path_vec.size()) v_fol = 0.05;
            else v_fol = turn_p.v_turn_90;
            a = turn_p.a_straight;

            switch (path_vec[i].turn_type) {
                case ETurnType::TURN_90: {
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.045, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsWallCenter(0.045, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                    }
                    break;
                case ETurnType::TURN_L_90: {
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.09, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsWallCenter(0.09, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                    }
                    break;
                case ETurnType::TURN_180: {
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.09, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsWallCenter(0.09, 0.0, v_fol, 0.05, a, a);
                        auto traj3 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj4 = StraightTrajectory::createAsWallCenter(0.09, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                        trajCommander.push(std::move(traj3));
                        trajCommander.push(std::move(traj4));

                    }
                    break;
                case ETurnType::TURN_S2D_45: {
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.045, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 45.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsDiagonal(0.045 * 1.4142356, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                    }

                    break;
                case ETurnType::TURN_S2D_135: {
                        auto traj0 = StraightTrajectory::createAsWallCenter(0.045, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 45.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsDiagonal(0.045*1.4142356, 0.0, v_fol, 0.05, a, a);
                        auto traj3 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj4 = StraightTrajectory::createAsDiagonal(0.045*1.4142356, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                        trajCommander.push(std::move(traj3));
                        trajCommander.push(std::move(traj4));
                    }

                    break;
                case ETurnType::TURN_D_90: {
                        auto traj0 = StraightTrajectory::createAsDiagonal(0.045 * 1.4142356, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsDiagonal(0.045 * 1.4142356, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                    }


                    break;
                case ETurnType::TURN_D2S_45: {
                        auto traj0 = StraightTrajectory::createAsDiagonal(0.045 * 1.4142356, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 45.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsWallCenter(0.045, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                    }

                    break;
                case ETurnType::TURN_D2S_135: {
                        auto traj0 = StraightTrajectory::createAsDiagonal(0.045 * 1.4142356, v_pre, v_pre, 0.05, a, a);
                        auto traj1 = SpinTurnTrajectory::create(dir * 90.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj2 = StraightTrajectory::createAsDiagonal(0.045 * 1.4142356, 0.0, v_fol, v_fol, a, a);
                        auto traj3 = SpinTurnTrajectory::create(dir * 45.0f, 2000.0f, 10000.0f);//pm.spin_ang_v, pm.spin_ang_a);
                        auto traj4 = StraightTrajectory::createAsWallCenter(0.045, 0.0, v_fol, v_fol, a, a);

                        trajCommander.push(std::move(traj0));
                        trajCommander.push(std::move(traj1));
                        trajCommander.push(std::move(traj2));
                        trajCommander.push(std::move(traj3));
                        trajCommander.push(std::move(traj4));

                    }

                    break;
            }
        }
    }
}
*/

void printPath(std::vector<Path>& path_vec) {
    PRINTF_ASYNC("==== path_vec ====\n");
    for (auto& path : path_vec) {
        path.print();
    }
    PRINTF_ASYNC("==================\n");
}
