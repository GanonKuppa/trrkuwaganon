#include "suction.h"

#include <string>
#include <cmath>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_pwm.h"

// Msg
#include "batteryInfoMsg.h"
#include "navStateMsg.h"
#include "msgBroker.h"


namespace module{
    
    Suction::Suction() :
    _duty(0.0f),  
    _target_duty(0.0f),
    _voltage(4.2f)
    {
        setModuleName("Suction");
        hal::setDutyPWM0(0.0f);
    }

    void Suction::updateEvery(){
        BatteryInfoMsg bat_msg;
        NavStateMsg nav_msg;
        copyMsg(msg_id::BATTERY_INFO, &bat_msg);
        copyMsg(msg_id::NAV_STATE, &nav_msg);
        _voltage = bat_msg.voltage_ave;
        

        // バッテリー残量が少なくなったことをファンを低速回転させることで通知
        bool is_low_voltage = bat_msg.is_low_voltage;        
        if(is_low_voltage && nav_msg.mode != ENavMode::FASTEST && nav_msg.mode != ENavMode::SEARCH){
            _target_duty = 0.1f;
        }
        // バッテリー電圧に応じたdutyをセット
        else if(_target_duty > 0.0f){
            constexpr float RATE_OF_CHANGE = 0.5f; // duty/s
            if( std::fabs(_duty - _target_duty) < _delta_t * RATE_OF_CHANGE * 2.0f) _duty = _target_duty; 
            else if(_duty < _target_duty) _duty += _delta_t * RATE_OF_CHANGE;
            else if(_duty > _target_duty) _duty -= _delta_t * RATE_OF_CHANGE;

            float max_voltage_duty = _duty * MAX_VOLTAGE / _voltage;
            hal::setDutyPWM0(max_voltage_duty);
        }
        
        else{
            _duty = 0.0f;
            _target_duty = 0.0f;
            hal::setDutyPWM0(0.0f);
        }        

    }

    void Suction::setDuty(float duty){
        _target_duty = duty;
    }

    float Suction::getDuty(){
        return _duty;
    }

    void Suction::debug(){
        PRINTF_ASYNC("  duty_r : %f\n", Suction::getInstance().getDuty());
    }

    int usrcmd_suction(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status             : print status\r\n");
            PRINTF_ASYNC("  duty  <0.0 to 1.0> : set duty (4.2V = 100%)\r\n");
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
