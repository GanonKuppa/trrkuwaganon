#pragma once

#include <stdint.h>
#include "baseModule.h"

namespace module {

    class Gamepad : public BaseModule<Gamepad> {
      public:
        
        void fetchCommand(uint8_t* command);
        void update0(); // 1msec毎に呼ぶこと
        void debug();
        void debug_xinput();

      private: 
        bool _connected;
        int8_t _cross_x_bt;
        int8_t _cross_y_bt;
        int8_t _L3D_x_bt;
        int8_t _L3D_y_bt;
        int8_t _R3D_x_bt;
        int8_t _R3D_y_bt;
        uint8_t _RT_bt;
        uint8_t _LT_bt;
        uint32_t _A_bt;
        uint32_t _B_bt;
        uint32_t _Y_bt;
        uint32_t _X_bt;
        uint32_t _RB_bt;
        uint32_t _LB_bt;
        uint32_t _BACK_bt;
        uint32_t _START_bt;
        
        uint64_t _preRecieveTime;
        uint8_t receiveCommand[16];
        Gamepad();
        bool _isConnected();
        void _publish();

        friend class BaseModule<Gamepad>;
    };

}

