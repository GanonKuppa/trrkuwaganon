#include "ledController.h"
#include "hal_gpio.h"
#include "led.h"
#include <memory>
#include <stdint.h>

namespace module {
    LedController::LedController() {
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

    void LedController::update0() {
        _led_r->update();
        _led_g->update();
        _led_b->update();
    }

    void LedController::turnFcled(bool r, bool g, bool b) {
        _led_r->turn(r);
        _led_g->turn(g);
        _led_b->turn(b);
    }

    uint8_t LedController::getFcledState() {
        return (_led_b->getState() << 2) + (_led_g->getState() << 1) +_led_r->getState();
    }

    void LedController::flashFcled(bool r, bool g, bool b, float on_time, float off_time) {
        if(r) _led_r->flash(on_time, off_time);
        else _led_r->turn(false);

        if(g) _led_g->flash(on_time, off_time);
        else _led_g->turn(false);

        if(b) _led_b->flash(on_time, off_time);
        else _led_b->turn(false);
    }

    int usrcmd_ledController(int argc, char **argv){
    	return 0;
    }
}
