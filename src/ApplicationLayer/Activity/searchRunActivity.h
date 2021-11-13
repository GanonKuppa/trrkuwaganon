#pragma once



#include <stdint.h>

#include "baseActivity.h"


namespace activity {

    class SearchRunActivity : public BaseActivity {
      public:
        std::string getModeName();
        void onStart();
        void onFinish();
        ELoopStatus loop();
      private:
        //ESearchMode _mode;
        float _a;
        float _v;
        float _v_max;

        void _slalom90(int8_t rot_times, uint8_t x_next, uint8_t y_next);
        void _straight(uint8_t x_next, uint8_t y_next);
        void _spin90(int8_t rot_times);
        void _spin180(int8_t rot_times);
        void _destiMove();


    };


}

