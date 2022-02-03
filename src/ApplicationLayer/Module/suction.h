#pragma once

#include "baseModule.h"

namespace module{

    class Suction : public BaseModule<Suction>{
      public:    
        void updateEvery();
        float getDuty();
        void setDuty(float duty);
        void debug();
      private:        
        float _duty;
        float _voltage;

        friend class BaseModule<Suction>;
        Suction();
    };

    int usrcmd_suction(int argc, char **argv);

}
