#pragma once

#include "baseModule.h"
#include <stdint.h>
#include <string>

namespace module {
    class ImuDriver : public BaseModule<ImuDriver> {
      public:
        void update0();
        void debug();
        void calibrateGyro(uint16_t num);
        void evalGyro(uint32_t num);
        void calibrateAcc(uint16_t num);
        void whoAmI();

      private:
        friend class BaseModule<ImuDriver>;
        
        int16_t _ang_v_raw[3];
        int16_t _acc_raw[3];
        
        float _ang_v_c[3];
        float _acc_c[3];

        float _ang_v_f[3];
        float _acc_f[3];
        float _acc_norm;

        float _ang_v_offset[3];
        float _acc_offset[3];        

        float _ang_v_f_int[3];

        int16_t _temp_raw;
        float _temp;

        std::string _imu_name;
        
        ImuDriver();
        void _writeReg(uint8_t adress, uint8_t data);
        uint8_t _readReg(uint8_t adress);
        void _publish();
    };

    int usrcmd_imuDriver(int argc, char **argv);
}
