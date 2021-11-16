#pragma once

#include "baseModule.h"
#include <deque>

#include "trajectory.h"

namespace module {
    class TrajectoryCommander : public BaseModule<TrajectoryCommander>{
      public:
        void update0();
        
        void push(std::unique_ptr<BaseTrajectory>&& traj);
        void clear();
        void reset(float x, float y, float yaw);
        void debug();


      private:
        friend class BaseModule<TrajectoryCommander>;
        TrajectoryCommander();

        float _x;
        float _y;
        float _yaw;

        std::deque< std::unique_ptr<BaseTrajectory> > _traj_queue;
        bool _lock_guard;

        void  _publish();
    };

    int usrcmd_trajectoryCommander(int argc, char **argv);

}
