#pragma once

#include <memory>
#include <unordered_map>


#include "baseModule.h"
#include "turnParameter.h"

//#include "trajectory.h"

enum class ETurnParams : uint8_t{
    SEARCH = 0,
    SAFE0,
    SAFE1,
    FAST0,
    FAST1,
    FAST2,
    FAST3,
    FAST4    
};


namespace module {
    class TrajectoryInitializer : public BaseModule<TrajectoryInitializer>{
      public:        
        float getV(ETurnParams tp, ETurnType tt);
        float getAcc(ETurnParams tp, ETurnType tt);
        float getPreDist(ETurnParams tp, ETurnType tt);
        float getFolDist(ETurnParams tp, ETurnType tt);
        float getTrajEndTime(ETurnParams tp, ETurnType tt);
        std::unique_ptr<TurnIterator> generateTurnIterator(ETurnParams tp, ETurnType tt);
        void debug();
      private:
        friend class BaseModule<TrajectoryInitializer>;
        TrajectoryInitializer();
        void _init();
        std::unordered_map<ETurnParams,TurnParameter> _turnParams;
        std::unordered_map<ETurnParams,TurnPreCalculations> _turnPreCalcs;

    };

    int usrcmd_trajectoryInitializer(int argc, char **argv);

    
}
