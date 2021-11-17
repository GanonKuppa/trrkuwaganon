#pragma once

#include "baseModule.h"
#include <deque>

// Msg
#include "navStateMsg.h"
#include "wallSensorMsg.h"

// Object
#include "maze.h"

namespace module {
    class Navigator : public BaseModule<Navigator> {
      public:
        
        void setNavMode(ENavMode mode);
        void setNavSubMode(ENavSubMode sub_mode);
        void startNavigation();
        void endNavigation();
        void update1();
        void updateInMainLoop();
        Maze& getMazeRef();                   
        void testPmap();
        
      private:
        friend class BaseModule<Navigator>;
        Navigator();
        
        Maze _maze;
        ENavMode _mode;
        ENavSubMode _sub_mode;
        bool _lock_guard;
        std::deque<ENavCommand> _nav_cmd_queue;
        bool _navigating;
        bool _done_outward;        
        
        uint8_t _x_cur;
        uint8_t _y_cur;
        EAzimuth _azimuth;
        
        uint8_t _x_dest;
        uint8_t _y_dest;

        uint8_t _x_goal;
        uint8_t _y_goal;
                
        bool _r_wall_enable;
        bool _l_wall_enable;
                
        bool _is_failsafe;
        bool _in_read_wall_area;
        
        WallSensorMsg _ws_msg;
        float _x;
        float _y;
        float _yaw;

        float _v;
        float _a;
        float _yawrate_max;
        float _yawacc;        
        float _wall2mouse_center_dist;

        float _read_wall_offset1;
        float _read_wall_offset2;

        void _updateParam();
        void _updateWallEnable();
        void _updateDestination();
        bool _isFailsafe();
        bool _inReadWallArea(float offset1, float offset2);        

        bool _existRWall(float x, float y, EAzimuth azimuth);
        bool _existLWall(float x, float y, EAzimuth azimuth);
        bool _watchedPillar(float x, float y, EAzimuth azimuth);
        
        void _publish();
        const float DEG2RAD = 3.14159265f / 180.0f;
    };

    int usrcmd_navigator(int argc, char **argv);

}
