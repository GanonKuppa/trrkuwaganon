#include "ledController.h"

#include <memory>
#include <stdint.h>

// Lib
#include "debugLog.h"
#include "ntlibc.h"

// Hal
#include "hal_gpio.h"

// Object
#include "led.h"


namespace module {
    LedController::LedController() :
    _oneshot_engaged(false),
    _oneshot_time(0.0f)
    {
        setModuleName("LedController");
        _led_r = std::make_unique<Led>(_delta_t, false, hal::setDout2);
        _led_g = std::make_unique<Led>(_delta_t, false, hal::setDout1);
        _led_b = std::make_unique<Led>(_delta_t, false, hal::setDout0);
        _led_r->turn(false);
        _led_g->turn(false);
        _led_b->turn(false);
    }

    void LedController::setDeltaT(float delta_t) {
        _delta_t = delta_t;
        _led_r->setDeltaT(delta_t);
        _led_g->setDeltaT(delta_t);
        _led_b->setDeltaT(delta_t);
    }

    void LedController::update1() {
        _led_r->update();
        _led_g->update();
        _led_b->update();

        _oneshot_time -= _delta_t;
        if(_oneshot_engaged && _oneshot_time < 0.0f){
            _oneshot_engaged = false;
            _led_r->turn(false);
            _led_g->turn(false);
            _led_b->turn(false);            
        }
        
        if(_oneshot_time < 0.0f){
            _oneshot_time = 0.0f;
        }
    }

    void LedController::turnFcled(bool r, bool g, bool b) {        
        if(_oneshot_engaged) return;
        _led_r->turn(r);
        _led_g->turn(g);
        _led_b->turn(b);
    }

    void LedController::oneshotFcled(bool r, bool g, bool b, float on_time, float off_time){
        if(_oneshot_engaged) return;
        LedController::flashFcled(r, g, b, on_time, off_time);
        _oneshot_engaged = true;
        _oneshot_time = on_time + off_time;        
    }


    uint8_t LedController::getFcledState() {
        return (_led_b->getState() << 2) + (_led_g->getState() << 1) +_led_r->getState();
    }

    void LedController::flashFcled(bool r, bool g, bool b, float on_time, float off_time) {
        if(_oneshot_engaged) return;

        if(r) _led_r->flash(on_time, off_time);
        else _led_r->turn(false);

        if(g) _led_g->flash(on_time, off_time);
        else _led_g->turn(false);

        if(b) _led_b->flash(on_time, off_time);
        else _led_b->turn(false);
    }

    void LedController::debug(){
        PRINTF_ASYNC(  "-- led_r --\n");
        _led_r->debug();
        PRINTF_ASYNC(  "-- led_g --\n");
        _led_g->debug();
        PRINTF_ASYNC(  "-- led_b --\n");
        _led_b->debug();
    }

    int usrcmd_ledController(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status                                   :\r\n");
            PRINTF_ASYNC("  oneshot <r> <g> <b> <on_time> <off_time> : call onshotFcled(r, g, b, on_time, off_time)\r\n");
            return 0;
        }


    	if (ntlibc_strcmp(argv[1], "status") == 0) {
            LedController::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "oneshot") == 0) {
            if(argc != 7){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            std::string r_str(argv[2]);
            std::string g_str(argv[3]);
            std::string b_str(argv[4]);
            std::string on_time_str(argv[5]);
            std::string off_time_str(argv[6]);
            bool r = std::stoi(r_str);
            bool g = std::stoi(g_str);
            bool b = std::stoi(b_str);
            float on_time = std::stof(on_time_str);
            float off_time = std::stof(off_time_str);
            LedController::getInstance().oneshotFcled(r, g, b, on_time, off_time);
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    }
}
