#pragma once

#include "baseModule.h"
#include <stdint.h>

namespace module {
    class ImuDriver : public BaseModule<ImuDriver> {
      public:
        void update0();
        void debug();
        void calibrateGyro(uint16_t num);
        void calibrateAcc(uint16_t num);

      private:
        
        uint16_t ang_v_raw[3];
        uint16_t acc_raw[3];
        
        uint16_t ang_v_c[3];
        uint16_t acc_c[3];

        float ang_v_f[3];
        float acc_f[3];

        uint16_t ang_v_offset[3];
        uint16_t acc_offset[3];


        float _temp;

        float gyro[3];
        float acc[3];
    }
}