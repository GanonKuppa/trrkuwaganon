#pragma once

#include "baseModule.h"
#include <deque>

// Msg
#include "navStateMsg.h"

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
        void updateSearchMap();
        
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
        
        uint8_t _x_dest;
        uint8_t _y_dest;

        uint8_t _x_goal;
        uint8_t _y_goal;
                
        bool _r_wall_enable;
        bool _l_wall_enable;
        
        EAzimuth _azimuth;
        bool _is_failsafe;
        bool _in_read_wall_area;
        WallSensorMsg _ws_msg;
        float _x;
        float _y;
        float _yaw;

        void _updateParam();
        void _publish();
        bool _inReadWallArea();
        void updateWall();

        bool _existRWall(float x, float y, EAzimuth azimuth);
        bool _existLWall(float x, float y, EAzimuth azimuth);
        bool _watchedPillar(float x, float y, EAzimuth azimuth);
        bool _isFailsafe();

    };

    int usrcmd_navigator(int argc, char **argv);

}
