#pragma once

#include "baseModule.h"
#include <stdint.h>

namespace module {
    class WheelOdometry : public BaseModule<WheelOdometry> {
      public:
      	void update0();                        
        void debug();
      private:
        int32_t _count_r;
        int32_t _count_l;
        float _ang_r;
        float _ang_l;
        float _v_r;
        float _v_l;
        float _v;
        float _v_ave;
        const float ENC_RES = 65536;

        WheelOdometry();
        void _publish();

        friend class BaseModule<WheelOdometry>;
    };

    int usrcmd_wheelOdometry(int argc, char **argv);
}
