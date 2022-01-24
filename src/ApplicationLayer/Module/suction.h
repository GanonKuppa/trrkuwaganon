#pragma once

#include "baseModule.h"

namespace module{

    class Suction : public BaseModule<Suction>{
      public:    
        void updateEvery();
        float getDuty();
        void setDuty(float duty);
        void useBuzzer(float time);
        void debug();
      private:
        float _duty;        
        float _voltage;
        float _buzzer_on_time;
        float _buzzer_save_duty;
        bool _used_buzzer;

        friend class BaseModule<Suction>;
        Suction();
    };

    int usrcmd_suction(int argc, char **argv);

}
