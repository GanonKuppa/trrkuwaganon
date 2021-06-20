#pragma once

class Led {

  public:
    Led(float delta_t, bool on_state, void (*setGpioFunc)(bool));
    void update();
    bool getState();
    void turn(bool state);
    void flash(float on_time, float off_time);
    void setDeltaT(float delta_t);

  private:
    bool _state;
    bool _on_state;
    float _led_time;
    float _on_time;
    float _off_time;
    float _delta_t;
    bool _flash_flag;
    void (*_setGpioFunc)(bool);
};
