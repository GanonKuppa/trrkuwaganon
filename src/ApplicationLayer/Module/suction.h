#pragma once

#include "baseModule.h"

namespace module{

    class Suction : public BaseModule<Suction>{
      public:    
        void update();
        float getDuty();
        void setDuty(float duty);        
      private:
        float _duty;
        float _on_time;
        const float MAX_ON_TIME = 20.0;

        friend class BaseModule<Suction>;
        Suction();
    };

    int usrcmd_suction(int argc, char **argv);

}
