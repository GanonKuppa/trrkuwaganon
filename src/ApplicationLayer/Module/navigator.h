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
        
        void updateSearchMap();
        void testPmap();
        

        Maze& getMazeRef();
      private:
        friend class BaseModule<Navigator>;
        Navigator();
        
        Maze _maze;
        ENavMode _mode;
        ENavSubMode _sub_mode;
        bool _lock_guard;
        std::deque<ENavCommand> _nav_cmd_queue;
        bool _navigating;
        bool _armed;
        int8_t _x_cur;
        int8_t _y_cur;

        int8_t _x_next;
        int8_t _y_next;
        
        int8_t _x_start;
        int8_t _y_start;

        int8_t _x_goal;
        int8_t _y_goal;
        
        bool _r_wall_enable;
        bool _l_wall_enable;
        
        float _dist_r;
        float _dist_l;
        float _dist_a;

        EAzimuth _azimuth;
        bool _is_failsafe;
        bool _in_read_wall_area;
        float _x;
        float _y;

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
