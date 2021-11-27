#pragma once

#include "stdint.h"
#include "maze.h"
#include "path.h"
#include "turnEnum.h"

#include <vector>

void setEntryArea(uint16_t start_x, uint16_t start_y, uint16_t goal_x, uint16_t goal_y, Maze& maze);
void makeMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec);
void makeQuasiMinStepPath(uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec);
void makeFastestDiagonalPath(uint32_t trial_times, ETurnParamSet tp, uint16_t goal_x, uint16_t goal_y, Maze& maze, std::vector<Path>& path_vec);


void translatePathSpin(std::vector<Path>& path_vec);
void translatePath90Deg(std::vector<Path>& path_vec);
void translatePathLong(std::vector<Path>& path_vec);
void translatePathDiagonal(std::vector<Path>& path_vec);
void HF_playPath(ETurnParamSet tp, std::vector<Path>& path_vec);
float HF_calcPlayPathTime(ETurnParamSet tp, std::vector<Path>& path_vec);
/*
void HF_playPathSpin(TurnParamSet tp, std::vector<Path>& path_vec);
void HF_playPathSpinDiagonal(TurnParamSet tp, std::vector<Path>& path_vec);
*/
void printPath(std::vector<Path>& path_vec);
