#include "suction.h"
#include "hal_pwm.h"

namespace module{
    
    void Suction::update(){
        if(_duty > 0.0f) _on_time += _delta_t;
        else _on_time = 0.0f;
    }

    void Suction::setDuty(float duty){
        _duty = duty;
        hal::setDutyPWM5(duty);
                
        if(_on_time > MAX_ON_TIME){
            _duty = 0.0f;
            hal::setDutyPWM5(0.0f);
        }
    }

    float Suction::getDuty(){
        return _duty;
    }

    Suction::Suction(){
        setModuleName("Suction");
        hal::setDutyPWM5(0.0f);
        _duty = 0.0f;
        _on_time = 0.0f;
    }
}
