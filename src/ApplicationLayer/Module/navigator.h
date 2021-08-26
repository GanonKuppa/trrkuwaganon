#pragma once

#include "baseModule.h"
#include "navStateMsg.h"

namespace module {
    class Navigator : public BaseModule<Navigator> {
      public:
        void update2();
        void setNavMode(ENavMode mode);
        void setNavSubMode(ENavSubMode mode);

      private:      
        friend class BaseModule<Navigator>;
        Navigator();
        ENavMode _mode;
        ENavSubMode _sub_mode;
        bool _armed;
        int8_t _x_cur;
        int8_t _y_cur;

        int8_t _x_next;
        int8_t _y_next;
        
        int8_t _x_goal;
        int8_t _y_goal;
        
        EAzimuth _azimuth;
        bool _is_failsafe;

        void _publish();

    };

    int usrcmd_navigator(int argc, char **argv);

}
