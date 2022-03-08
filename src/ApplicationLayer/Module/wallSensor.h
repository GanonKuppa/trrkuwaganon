#pragma once

#include "baseModule.h"
#include "led.h"

#include <deque>
#include <stdint.h>

namespace module {
    class WallSensor : public BaseModule<WallSensor> {
      public:
      	void update0();      	     
        void setEnable(bool en);
        void emitLedTask();
        void debug();
        void eval();
        void printCycleTime();
      private:
        bool _enable;
        int16_t _ahead_l_on;
        int16_t _ahead_r_on;
        int16_t _left_on;
        int16_t _right_on;

        int16_t _ahead_l_off;
        int16_t _ahead_r_off;
        int16_t _left_off;
        int16_t _right_off;

        std::deque<int16_t> _ahead_l_q;
        std::deque<int16_t> _ahead_r_q;
        std::deque<int16_t> _left_q;
        std::deque<int16_t> _right_q;

        float _dist_al;
        float _dist_ar;
        float _dist_a;
        float _dist_l;
        float _dist_r;

        bool _is_ahead_l;
        bool _is_ahead_r;
        bool _is_ahead;
        bool _is_left;
        bool _is_right;
        bool _is_left_ctrl;
        bool _is_right_ctrl;        

        bool _is_contact_wall;
        bool _is_on_wall_center;

        bool _is_corner_l;
        bool _is_corner_r;
        bool _is_diag_corner_l;
        bool _is_diag_corner_r;


        float _contact_wall_time;        
        float _on_wall_ahead_time;
        float _on_wall_ahead_l_time;
        float _on_wall_ahead_r_time;
        float _on_wall_center_time;
        float _not_corner_l_elapsed_time;
        float _not_corner_r_elapsed_time;

        const uint8_t BUFF_SIZE = 25;
        const uint8_t LED_ON_SEC = 100;        
        float _emit_led_cycle_time_us[8];

        WallSensor();
        void _updateOffVal(bool sled1, bool sled2, bool sled3, bool sled4);
        void _updateOnVal(bool sled1, bool sled2, bool sled3, bool sled4);
        void _turnLed(bool sled1, bool sled2, bool sled3, bool sled4);
        void _modulateVal();
        void _publish();

        float _aheadDistL(float ad);
        float _aheadDistR(float ad);
        float _distL(float ad);
        float _distR(float ad);
        float _aheadDist(float dist_al, float dist_ar);
        bool _isCornerL();
        bool _isCornerR();
        bool _isDiagCornerL();
        bool _isDiagCornerR();
        int16_t _leftMax();
        int16_t _rightMax();

        uint16_t _trimAverage(uint16_t ad_array[], uint16_t num, uint16_t trim_num);

        friend class BaseModule<WallSensor>;
    };

    int usrcmd_wallSensor(int argc, char **argv);
}
