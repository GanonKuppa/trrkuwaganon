#pragma once

#include "path.h"
#include <stdint.h>
#include <vector>

void compress_l_90(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_180(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_s2d_135(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_d2s_135(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_s2d_45(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_d2s_45(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_d_90(std::vector<Path>& path_vec, uint16_t start_index=0);
void compress_straight(std::vector<Path>& path_vec, uint16_t start_num=0);
void compress_d_straight(std::vector<Path>& path_vec, uint16_t start_index=0);
