#include <stdio.h>

#ifdef SILS
#include <XInput.h>
#include <windows.h>
#endif

#include "hal_timer.h"

#include "gamepad.h"
#include "gamepadMsg.h"
#include "msgBroker.h"

namespace module {


    Gamepad::Gamepad():
        _connected(false),
        _cross_x_bt(0),
        _cross_y_bt(0),
        _L3D_x_bt(0),
        _R3D_x_bt(0),
        _R3D_y_bt(0),
        _RT_bt(0),
        _LT_bt(0),
        _A_bt(0),
        _B_bt(0),
        _Y_bt(0),
        _X_bt(0),
        _RB_bt(0),
        _LB_bt(0),
        _BACK_bt(0),
        _START_bt(0),        
        _preRecieveTime(0)
    {
        setModuleName("Gamepad");
    }



    bool Gamepad::_isConnected() {
#ifndef SILS
        if(hal::getElapsedMsec() - _preRecieveTime < 200) {
            return true;
        } else {
            return false;
        }
#else
        static XINPUT_CAPABILITIES dummy;
        if(XInputGetCapabilities(0,0, &dummy) == 0) {
            return true;
        } else {
            return false;
        }
#endif

    }


    void Gamepad::fetchCommand(uint8_t* command) {
        if(command[3] == 254 && command[4] == 253) {
            _preRecieveTime = hal::getElapsedMsec();
            for(uint8_t i=0; i<16; i++) {
                receiveCommand[i] = command[i];
            }
        }
    };

    //1msec毎に呼ぶこと
    void Gamepad::update0() {
#ifndef SILS
        uint8_t* command = receiveCommand;
        if((command[6] & 0x01) == 1) _A_bt++;
        else _A_bt = 0;
        if(((command[6] & 0x02) >> 1) == 1) _B_bt++;
        else _B_bt = 0;
        if(((command[6] & 0x04) >> 2) == 1) _X_bt++;
        else _X_bt = 0;
        if(((command[6] & 0x08) >> 3) == 1) _Y_bt++;
        else _Y_bt = 0;
        if(((command[6] & 0x10) >> 4) == 1) _RB_bt++;
        else _RB_bt = 0;
        if(((command[6] & 0x20) >> 5) == 1) _LB_bt++;
        else _LB_bt = 0;
        if(((command[6] & 0x40) >> 6) == 1) _BACK_bt++;
        else _BACK_bt = 0;
        if(((command[6] & 0x80) >> 7) == 1) _START_bt++;
        else _START_bt = 0;
        _RT_bt = command[7];
        _LT_bt = command[8];
        _cross_x_bt = command[9]  - 128;
        _cross_y_bt = command[10] - 128;
        _R3D_x_bt = command[11] - 128;
        _R3D_y_bt = command[12] - 128;
        _L3D_x_bt = command[13] - 128;
        _L3D_y_bt = command[14] - 128;
#else
        _preRecieveTime = hal::getElapsedMsec();
        XINPUT_STATE state;
        XInputGetState(0, &state);
        uint16_t wButtons = state.Gamepad.wButtons;
        if((wButtons & 0x1000) == 0x1000) _A_bt++;
        else _A_bt = 0;
        if((wButtons & 0x2000) == 0x2000) _B_bt++;
        else _B_bt = 0;
        if((wButtons & 0x4000) == 0x4000) _X_bt++;
        else _X_bt= 0;
        if((wButtons & 0x8000) == 0x8000) _Y_bt++;
        else _Y_bt = 0;
        if((wButtons & 0x0200) == 0x0200) _RB_bt++;
        else _RB_bt = 0;
        if((wButtons & 0x0100) == 0x0100) _LB_bt++;
        else _LB_bt = 0;
        if((wButtons & 0x0020) == 0x0020) _BACK_bt++;
        else _BACK_bt = 0;
        if((wButtons & 0x0010) == 0x0010) _START_bt++;
        else _START_bt = 0;

        _RT_bt = state.Gamepad.bRightTrigger;
        _LT_bt = state.Gamepad.bLeftTrigger;

        if((wButtons & 0x0004) == 0x0004) _cross_x_bt = 1;
        else if((wButtons & 0x0008) == 0x0008) _cross_x_bt = -1;
        else _cross_x_bt = 0;

        if((wButtons & 0x0001) == 0x0001) _cross_y_bt = 1;
        else if((wButtons & 0x0002) == 0x0002) _cross_y_bt = -1;
        else _cross_y_bt = 0;

        _R3D_x_bt = state.Gamepad.sThumbRX / 256;
        _R3D_y_bt = state.Gamepad.sThumbRY / 256;
        _L3D_x_bt = state.Gamepad.sThumbLX / 256;
        _L3D_y_bt = state.Gamepad.sThumbLY / 256;

#endif
        _connected = _isConnected();
        if(!_connected) {
            _A_bt = 0;
            _B_bt = 0;
            _X_bt = 0;
            _Y_bt = 0;
            _RB_bt = 0;
            _LB_bt = 0;
            _BACK_bt = 0;
            _START_bt = 0;
            _RT_bt = 0;
            _LT_bt = 0;
            _cross_x_bt = 0;
            _cross_y_bt = 0;
            _R3D_x_bt = 0;
            _R3D_y_bt = 0;
            _L3D_x_bt = 0;
            _L3D_y_bt = 0;
        }

        _publish();
    };

    void Gamepad::_publish() {
        GamepadMsg msg;
        msg.connected = _connected;
        msg.A = _A_bt;
        msg.B = _B_bt;
        msg.X = _X_bt;
        msg.Y = _Y_bt;
        msg.RB = _RB_bt;
        msg.LB = _LB_bt;
        msg.BACK = _BACK_bt;
        msg.START = _START_bt;
        msg.RT = _RT_bt;
        msg.LT = _LT_bt;
        msg.cross_x = _cross_x_bt;
        msg.cross_y = _cross_y_bt;
        msg.L3D_x = _L3D_x_bt;
        msg.L3D_y = _L3D_y_bt;
        msg.R3D_x = _R3D_x_bt;
        msg.R3D_y = _R3D_y_bt;
        publishMsg(msg_id::GAMEPAD, &msg);
    };



    void Gamepad::debug() {
#ifndef SILS
        //printfAsync("A:%d, B:%d, X:%d, Y:%d, RT:%d, LT:%d  \n", A,B,X,Y,RT,LT);
#else
        printf("==================\n");
        printf("A:%d, B:%d, X:%d, Y:%d, RB:%d, LB:%d  \n", _A_bt,_B_bt,_X_bt,_Y_bt,_RB_bt,_LB_bt);
        printf("cross_x:%d, cross_y:%d \n", _cross_x_bt, _cross_y_bt);
        printf("R3D_x:%d, R3D_y:%d, L3D_x:%d, L3D_y:%d, RT:%d, LT:%d  \n", _R3D_x_bt,_R3D_y_bt,_L3D_x_bt,_L3D_y_bt,_RT_bt,_LT_bt);
#endif
    };

    void Gamepad::debug_xinput() {
#ifdef SILS
        XINPUT_STATE state;
        XInputGetState(0, &state);
        printf("==================\n");
        printf("wButtons: %x \n", state.Gamepad.wButtons);
        printf("sThumbLX: %d \n", state.Gamepad.sThumbLX);
        printf("sThumbLY: %d \n", state.Gamepad.sThumbLY);
        printf("sThumbRX: %d \n", state.Gamepad.sThumbRX);
        printf("sThumbRY: %d \n", state.Gamepad.sThumbRY);
        printf("bRightTrigger: %d \n", state.Gamepad.bRightTrigger);
        printf("bLeftTrigger: %d \n", state.Gamepad.bLeftTrigger);
#endif
    }

}
