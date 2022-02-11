#pragma once

#include "baseMsg.h"

class WallSensorMsg : public BaseMsg{
  public:  
    int16_t ahead_l = 0;               // [12bit AD]
    int16_t ahead_r = 0;               // [12bit AD]
    int16_t left = 0;                  // [12bit AD]
    int16_t right = 0;                 // [12bit AD]
    
    float dist_al = 0.0f;              // [m]
    float dist_ar = 0.0f;              // [m]
    float dist_a = 0.0f;               // [m]
    float dist_l = 0.0f;               // [m]
    float dist_r = 0.0f;               // [m]

    bool is_ahead_l = false;           // [bool]
    bool is_ahead_r = false;           // [bool]
    bool is_ahead = false;             // [bool]
    bool is_left = false;              // [bool]
    bool is_right = false;             // [bool]
    bool is_left_ctrl = false;         // [bool]
    bool is_right_ctrl = false;        // [bool]

    bool is_contact_wall = false;      // [bool]
    bool is_on_wall_center = false;    // [bool]        

    bool is_corner_l = false;          // [bool]
    bool is_corner_r = false;          // [bool]
    bool is_diag_corner_l = false;     // [bool]
    bool is_diag_corner_r = false;     // [bool]

    float contact_wall_time = 0.0f;    // [s]
    float on_wall_ahead_time = 0.0f;   // [s]
    float on_wall_ahead_l_time = 0.0f; // [s]
    float on_wall_ahead_r_time = 0.0f; // [s]
    float on_wall_center_time = 0.0f;  // [s]
    float not_corner_l_elapsed_time = 0.0f; // [s]
    float not_corner_r_elapsed_time = 0.0f; // [s]
};