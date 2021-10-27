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


      private:                
        friend class BaseModule<TrajectoryCommander>;        
        TrajectoryCommander();

        float _x;
        float _v_x;
        float _a_x;
        float _y;
        float _v_y;
        float _a_y;
        float _v_xy_body;        
        float _a_xy_body;
        float _yaw;
        float _yawrate;
        float _yawacc;
        float _beta;
        float _beta_dot;

        ETrajType _traj_type;
        ETurnType _turn_type;
        ETurnDir _turn_dir;

        std::deque< std::unique_ptr<BaseTrajectory> > _traj_queue;        
        bool _lock_guard;

        void  _publish();        
    };

    int usrcmd_trajectoryCommander(int argc, char **argv);

}
