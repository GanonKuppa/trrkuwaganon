#pragma once

#include <stdint.h>
#include "baseModule.h"

namespace module {

    class Gamepad : public BaseModule<Gamepad> {
      public:
        
        void fetchCommand(uint8_t* command);
        void update(); // 1msec毎に呼ぶこと
        void debug();
        void debug_xinput();

      private: 
        bool _connected;
        int8_t _cross_x;
        int8_t _cross_y;
        int8_t _L3D_x;
        int8_t _L3D_y;
        int8_t _R3D_x;
        int8_t _R3D_y;
        uint8_t _RT;
        uint8_t _LT;
        uint32_t _A;
        uint32_t _B;
        uint32_t _Y;
        uint32_t _X;
        uint32_t _RB;
        uint32_t _LB;
        uint32_t _BACK;
        uint32_t _START;
        
        uint64_t _preRecieveTime;
        uint8_t receiveCommand[16];
        Gamepad();
        bool _isConnected();
        void _publish();

        friend class BaseModule<Gamepad>;
    };

}

