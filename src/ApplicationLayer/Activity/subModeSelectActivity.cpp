#include "subModeSelectActivity.h"

// Module
#include "ledController.h"
#include "pseudoDial.h"

// Msg
#include "msgBroker.h"
#include "dialPositionMsg.h"
#include "wallSensorMsg.h"
#include "gamepadMsg.h"



namespace activity{    


    std::string SubModeSelectActivity::getModeName()
    {
        std::string mode_name("SubModeSelectActivity");
        return mode_name;
    }

    void SubModeSelectActivity::onStart(){
        if(_intent.uint8_t_param.count("SUB_MODE_NUM") == 1) {
            _mode_num = _intent.uint8_t_param["SUB_MODE_NUM"];
        } else {
            _mode_num = 8;
        }

        if(_intent.float_param.count("LED_ON_SEC") == 1) {
            _led_on_sec = _intent.float_param["LED_ON_SEC"];
        } else {
            _led_on_sec = 0.25f;
        }

        if(_intent.float_param.count("LED_OFF_SEC") == 1) {
            _led_off_sec = _intent.float_param["LED_OFF_SEC"];
        } else {
            _led_off_sec = 0.25f;
        }

        ModeSelectActivity::onStart();
    }
    
    
    void SubModeSelectActivity::onFinish(){
        module::PseudoDial& pd = module::PseudoDial::getInstance();
        pd.reset();
        pd.setEnable(false);

        _intent.uint8_t_param["SUB_MODE"] = _mode;
        PRINTF_ASYNC("sub mode select finish. submode = %d\n",_mode);
        module::LedController::getInstance().turnFcled(0, 0, 0);
    }


    SubModeSelectActivity::ELoopStatus SubModeSelectActivity::loop() {

        GamepadMsg gp_msg;
        DialPositionMsg dp_msg;
        WallSensorMsg ws_msg;

        copyMsg(msg_id::GAMEPAD, &gp_msg);
        copyMsg(msg_id::DIAL_POSITION, &dp_msg);
        copyMsg(msg_id::WALL_SENSOR, &ws_msg);
        uint8_t mode_pre = _mode;

        if(gp_msg.connected){
            if(gp_msg.cross_y == 1){
                _mode =  (_mode + _mode_num + 1) % _mode_num;
                hal::waitmsec(100);
                //sound::cursor_move();
            }
            
            if(gp_msg.cross_y == -1){
                _mode =  (_mode + _mode_num - 1) % _mode_num;
                hal::waitmsec(100);
                //sound::cursor_move();
            }

            if(gp_msg.B > 2000){
                hal::waitmsec(100);
                //sound::confirm();
                return ELoopStatus::FINISH;
            }
        }
        else{
            _mode = dp_msg.dial_pos_l;

            if( (dp_msg.dial_pos_r  == 4 && dp_msg.same_pos_time_r > 0.1f ) || ws_msg.on_wall_ahead_time > 1.0f) {
                return ELoopStatus::FINISH;
            }

        } 

        if(_mode != mode_pre){
            _turnFcled(_mode);
        }

        return ELoopStatus::CONTINUE;
    }

    void SubModeSelectActivity::_turnFcled(uint8_t mode) {
        module::LedController& fcled = module::LedController::getInstance();

        if      (mode == 0) fcled.turnFcled(0, 0, 0);                             // BLACK
        else if (mode == 1) fcled.flashFcled(1, 0, 0, _led_on_sec, _led_off_sec); // RED
        else if (mode == 2) fcled.flashFcled(0, 1, 0, _led_on_sec, _led_off_sec); // GREEN
        else if (mode == 3) fcled.flashFcled(1, 1, 0, _led_on_sec, _led_off_sec); // YELLOW
        else if (mode == 4) fcled.flashFcled(0, 0, 1, _led_on_sec, _led_off_sec); // BLUE
        else if (mode == 5) fcled.flashFcled(1, 0, 1, _led_on_sec, _led_off_sec); // MAGENTA
        else if (mode == 6) fcled.flashFcled(0, 1, 1, _led_on_sec, _led_off_sec); // CYAN
        else if (mode == 7) fcled.flashFcled(1, 1, 1, _led_on_sec, _led_off_sec); // WHITE
    }


}

