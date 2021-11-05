#include "led.h"

#include "debugLog.h"

Led::Led(float delta_t, bool on_state, void (*setGpioFunc)(bool)):
    _state(false),
    _led_time(0.0f),
    _on_time(0.0f),
    _off_time(0.0f),
    _delta_t(delta_t),
    _on_state(on_state),
    _flash_flag(false),
    _setGpioFunc(setGpioFunc)
{}

void Led::update() {
    if (_flash_flag) {
        if (_led_time < _on_time) {
            _setGpioFunc(true);
            _state = true;
        } else {
            _setGpioFunc(false);
            _state = false;
        }

        if ((_on_time + _off_time) < _led_time){
            _led_time = 0.0f;
        }
        _led_time += _delta_t;
    }
}

bool Led::getState() {
    return _state;
}

void Led::turn(bool state) {
    _state = state;
    _flash_flag = false;
    _led_time = 0.0f;
    if(_on_state) _setGpioFunc(state);
    else _setGpioFunc(!state);
}

void Led::flash(float on_time, float off_time) {
    _flash_flag = true;
    _led_time = 0.0f;
    _on_time = on_time;
    _off_time = off_time;
}

void Led::setDeltaT(float delta_t) {
    _delta_t = delta_t;
}

void Led::debug(){    
    PRINTF_ASYNC("  state      : %d\n", _state);
    PRINTF_ASYNC("  on_state   : %d\n", _on_state);
    PRINTF_ASYNC("  led_time   : %f\n", _led_time);
    PRINTF_ASYNC("  on_time    : %f\n", _on_time);
    PRINTF_ASYNC("  off_time   : %f\n", _off_time);
    PRINTF_ASYNC("  delta_t    : %f\n", _delta_t);
    PRINTF_ASYNC("  flash_flag : %d\n", _flash_flag);
}
