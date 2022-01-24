#include "suction.h"

#include <string>

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
    _voltage(4.2f),
    _buzzer_on_time(0.0f),
    _buzzer_save_duty(0.0f),
    _used_buzzer(false)
    {
        setModuleName("Suction");
        hal::setDutyPWM0(0.0f);
    }

    void Suction::useBuzzer(float time){
        if(_used_buzzer) return;
        _buzzer_on_time = time;
        _used_buzzer = true;
        _buzzer_save_duty = _duty;
    }

    void Suction::updateEvery(){
        BatteryInfoMsg bat_msg;
        NavStateMsg nav_msg;
        copyMsg(msg_id::BATTERY_INFO, &bat_msg);
        copyMsg(msg_id::NAV_STATE, &nav_msg);
        _voltage = bat_msg.voltage;

        // ファンをブザーとして使用する場合の処理
        if(_used_buzzer && _buzzer_on_time > 0.0f){
            _duty = 0.7f;
            setDuty(_duty);
        }
        
        if(_used_buzzer){
            _buzzer_on_time -= _delta_t;
        }
        else{
            _buzzer_on_time = 0.0f;
        }

        if(_buzzer_on_time < 0.0f){
            _buzzer_on_time = 0.0f;
            _duty = _buzzer_save_duty;
            setDuty(_duty);
            _used_buzzer = false;
        }

        // バッテリー残量が少なくなったことをファンを低速回転させることで通知
        bool is_low_voltage = bat_msg.is_low_voltage;        
        if(is_low_voltage && nav_msg.mode != ENavMode::FASTEST && nav_msg.mode != ENavMode::SEARCH){
            setDuty(0.1f);
        }        

    }

    void Suction::setDuty(float duty){
        _duty = duty * _voltage / 4.2f;
        hal::setDutyPWM0(duty);                
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
            PRINTF_ASYNC("  buzzer <float val> : buzzer test\r\n");
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

        if (ntlibc_strcmp(argv[1], "buzzer") == 0){
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }

            std::string time_val_str(argv[2]);
            float time_val = std::stof(time_val_str);
            Suction::getInstance().useBuzzer(time_val);
            return 0;
        }


        module::Suction::getInstance().useBuzzer(0.1f);

        PRINTF_ASYNC("  Unknown sub command found\r\n");            
        return -1;
    }
}
