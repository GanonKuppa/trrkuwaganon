#include "led.h"


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
    if (_flash_flag == true) {
        if (_led_time < _on_time) {
            _setGpioFunc(true);
            _state = true;
        } else {
            _setGpioFunc(false);
            _state = false;
        }

        if ((_on_time + _off_time) < _led_time) _led_time = 0.0f;
        _led_time += _delta_t;
    }
}

bool Led::getState() {
    return _state;
}

void Led::turn(bool state) {
    _state = state;
    _flash_flag = false;
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
