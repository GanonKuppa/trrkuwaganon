#pragma once

#include "baseModule.h"

namespace module{

    class Suction : public BaseModule<Suction>{
    public:    
        void update();
        float getDuty();
        void setDuty();
        void setDeltaT(float delta_t);
    private:
        float _duty;

        friend class BaseModule<Suction>;
        Suction();
    };

}
