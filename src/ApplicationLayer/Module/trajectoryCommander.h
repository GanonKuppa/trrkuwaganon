#pragma once

#include "baseModule.h"
//#include "trajectory.h"

namespace module {
    class TrajectoryCommander : public BaseModule<TrajectoryCommander>{
      
      private:
        friend class BaseModule<TrajectoryCommander>;
        
        TrajectoryCommander();
        void  _publish();
        void _initTraj();
    };

    int usrcmd_trajectoryCommander(int argc, char **argv);

}
