#pragma once

#include "baseModule.h"
#include <deque>

// Msg
#include "navStateMsg.h"
#include "wallSensorMsg.h"
#include "actuatorOutputMsg.h"
#include "ctrlSetpointMsg.h"
#include "trajTripletMsg.h"

// Object
#include "maze.h"
#include "turnEnum.h"

namespace module {
    class Navigator : public BaseModule<Navigator> {
      public:
        
        void setNavMode(ENavMode mode);
        void setNavSubMode(ENavSubMode sub_mode);
        void startNavigation();
        void endNavigation();
        void update0();
        void updateInMainLoop();
        Maze& getMazeRef();              
        void testPmap();
        void printMaze();
        void debugWall(uint8_t x, uint8_t y);
        
      private:
        friend class BaseModule<Navigator>;
        Navigator();
        
        Maze _maze;
        ENavMode _mode;
        ENavSubMode _sub_mode;
        ECtrlMode _ctrl_mode;
        bool _lock_guard;
        std::deque<ENavCommand> _nav_cmd_queue;
        bool _navigating;
        bool _done_outward;
        float _elapsed_time;
        float _search_limit_time;
        uint32_t _cumulative_section_count;
        uint32_t _dusty_tire_section_count;
        
        uint8_t _x_cur;
        uint8_t _y_cur;
        uint8_t _x_last;
        uint8_t _y_last;
        EAzimuth _azimuth;
        
        uint8_t _x_dest;
        uint8_t _y_dest;

        uint8_t _x_goal;
        uint8_t _y_goal;
                
        bool _r_wall_enable;
        bool _l_wall_enable;
        
        bool _is_actuator_error;
        bool _is_failsafe;
        bool _in_read_wall_area;
        bool _in_reread_ahead_area;
        
        WallSensorMsg _ws_msg;
        TrajTripletMsg _traj_msg;
        float _x;
        float _y;
        float _yaw;

        float _x_setp;
        float _y_setp;
        float _yaw_setp;
        float _v_setp;
        ETurnType _turn_type;

        float _v;
        float _v_max;
        float _a;
        float _yawrate_max;
        float _yawacc;        
        float _wall2mouse_center_dist;
        ETurnParamSet _turn_param_set;

        float _read_wall_offset1;
        float _read_wall_offset2;
        bool _is_pre_read_l_wall;
        bool _is_pre_read_r_wall;
        float _pre_read_l_wall_dist;
        float _pre_read_r_wall_dist;
        uint8_t _crossroads_count;
        uint8_t _slalom_count;

        void _updateParam();
        void _updateWallEnable();
        void _updateDestination();
        bool _isFailsafe();
        bool _inReadWallArea(float offset_pre, float offset_fol);

        bool _existsRWall(float x, float y, EAzimuth azimuth);
        bool _existsLWall(float x, float y, EAzimuth azimuth);
        bool _watchedPillar(float x, float y, EAzimuth azimuth);
        void _updateRunParameter();
        void _printWall();
        
        void _publish();
        const float DEG2RAD = 3.14159265f / 180.0f;

    };

    int usrcmd_navigator(int argc, char **argv);

}
