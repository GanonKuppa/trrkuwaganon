#pragma once

#include "baseModule.h"
#include <stdint.h>

namespace module {
    class WheelOdometry : public BaseModule<WheelOdometry> {
      public:
      	void update0();                        
        void debug();
        void evalAng(float duty);
        void evalVelocity(float duty);
      private:
        int32_t _count_r;
        int32_t _count_l;
        int32_t _count_r_pre;
        int32_t _count_l_pre;

        float _ang_r;
        float _ang_l;

        float _rpm_r;
        float _rpm_l;
        
        float _ang_r_cor;
        float _ang_l_cor;

        float _v_r;
        float _v_l;
        float _v;
        float _v_ave;

        float _ang_v_rad;
        float _ang_v;

        const float ENC_RES = 65536;
        const float ENC_R_DIR = -1.0f;
        const float ENC_L_DIR =  1.0f;
        const float PI = 3.141592653589f;
        const float TIRE_GEAR_NUM = 33.0;
        const float ENC_GEAR_NUM = 33.0;
        const float GEAR_RATIO = TIRE_GEAR_NUM / ENC_GEAR_NUM;

        float _dia_tire;
        float _tread;


        WheelOdometry();
        void _publish();
        void _readEncoder();
        void _updateParam();
        

        friend class BaseModule<WheelOdometry>;
    };

    int usrcmd_wheelOdometry(int argc, char **argv);
}
