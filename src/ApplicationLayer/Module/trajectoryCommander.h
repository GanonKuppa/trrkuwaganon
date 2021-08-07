#pragma once

#include "baseModule.h"
#include "trajectory.h"

namespace module {
    class TrajectoryCommander {
      public:
        void update0(){
            if(_lock_guard) return;

            if(!trajQueue.empty()) {
                trajQueue.front()->update();
            }

            if(trajQueue.front()->isEnd()) {
                _x = trajQueue.front()->getEndX();
                _y = trajQueue.front()->getEndY();
                _ang = trajQueue.front()->getEndAng();
                trajQueue.pop_front();
            }
        };
        void debug();
      
      private:
        friend class BaseModule<TrajectoryCommander>;
        EMotionType motion_type;
        turn_type_e turn_type;
        uint16_t traj_hash;
        std::deque< std::unique_ptr<BaseTrajectory> > trajQueue;        
        bool _lock_guard;

        void  _publish()


    }



}