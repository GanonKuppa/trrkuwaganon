#pragma once

#include <stdint.h>
#include <deque>

#include "baseModule.h"

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

        float _ang_r_deg;
        float _ang_l_deg;

        float _rpm_r;
        float _rpm_l;
        float _rpm_r_ave;
        float _rpm_l_ave;


        float _ang_r_cor;
        float _ang_l_cor;

        float _v_r;
        float _v_l;
        float _v_r_ave;
        float _v_l_ave;

        float _v;        

        float _yawrate_rad;
        float _yawrate_deg;

        const float ENC_RES = 65536;
        const float ENC_R_DIR = -1.0f;
        const float ENC_L_DIR =  1.0f;
        const float PI = 3.141592653589f;
        const float TIRE_GEAR_NUM = 33.0;
        const float ENC_GEAR_NUM = 33.0;
        const float GEAR_RATIO = TIRE_GEAR_NUM / ENC_GEAR_NUM;

        float _dia_tire;
        float _tread;

        const uint8_t AVERAGE_NUM = 30;
        std::deque<float> _v_r_list;
        std::deque<float> _v_l_list;

        WheelOdometry();
        void _publish();
        void _readEncoder();
        void _updateParam();
        

        friend class BaseModule<WheelOdometry>;
    };

    int usrcmd_wheelOdometry(int argc, char **argv);
}
