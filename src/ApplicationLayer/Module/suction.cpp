#include "suction.h"

#include <string>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_pwm.h"



namespace module{
    
    void Suction::update(){
        if(_duty > 0.0f) _on_time += _delta_t;
        else _on_time = 0.0f;
    }

    void Suction::setDuty(float duty){
        _duty = duty;
        hal::setDutyPWM0(duty);
                
        if(_on_time > MAX_ON_TIME){
            _duty = 0.0f;
            hal::setDutyPWM0(0.0f);
        }
    }

    float Suction::getDuty(){
        return _duty;
    }

    Suction::Suction(){
        setModuleName("Suction");
        hal::setDutyPWM0(0.0f);
        _duty = 0.0f;
        _on_time = 0.0f;
    }

    void Suction::debug(){
        PRINTF_ASYNC("  duty_r : %f\n", Suction::getInstance().getDuty());
    }

    int usrcmd_suction(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status            :\r\n");
            PRINTF_ASYNC("  duty <0.0 to 1.0> :\r\n");
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            Suction::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "duty") == 0){
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }

            std::string duty_val_str(argv[2]);
            float duty_val = std::stof(duty_val_str);
            Suction::getInstance().setDuty(duty_val);
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");            
        return -1;
    }
}
