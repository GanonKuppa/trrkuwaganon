#include "heater.h"
#include "pidController.h"

#include "parameterManager.h"

#include "imuMsg.h"
#include "batteryInfoMsg.h"
#include "msgBroker.h"

#include "hal_pwm.h"
#include "ntlibc.h"


namespace module{
    Heater::Heater() :
    _duty(0.0f),
    _temp(25.0f),
    _current_ave(0.0f),
    _current_1sec_sum(0.0f),
    _current_time(0.0f),
    _voltage(0.0f)
    {
        setModuleName("Heater");
        ParameterManager& pm = ParameterManager::getInstance();
        _p = pm.heater_p;
        _i = pm.heater_i;
        _i_limit = pm.heater_i_limit;
        _limit = pm.heater_limit;
        _target_temp = pm.heater_target_temp;             
        _pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
        _pidf.setEnable(true);
        hal::setDutyPWM0(0.0f);

    }

    float Heater::getDuty(){
        return _duty;
    }

    void Heater::setTargetTemp(float temp){
        _target_temp = temp;
        ParameterManager& pm = ParameterManager::getInstance();
        pm.write<float>("heater_target_temp", temp);
    }    

    void Heater::updateEvery(){
    	ParameterManager& pm = ParameterManager::getInstance();
        
        ImuMsg imu_msg;
        copyMsg(msg_id::IMU, &imu_msg);
        _temp = imu_msg.temp;

        BatteryInfoMsg _vol_msg;
        copyMsg(msg_id::BATTERY_INFO, &_vol_msg);
        _voltage = _vol_msg.voltage;

        _p = pm.heater_p;
        _i = pm.heater_i;
        _i_limit = pm.heater_i_limit;
        _limit = pm.heater_limit;
        _target_temp = pm.heater_target_temp;
        _pidf.set(_p, _i, 0.0f, 0.0f);
        _pidf.setSaturationEnable(true);
        _pidf.setSaturation(_limit);
        _pidf.setIntegralSaturationEnable(true);
        _pidf.setIntegralSaturation(_i_limit);
        _pidf.update(_target_temp, _temp);
        _duty = _pidf.getControlVal();
        if(_duty < 0.0f){
            _duty = 0.0f;
            _pidf.reset();
        }
        hal::setDutyPWM0(_duty);
        
        _current_1sec_sum += _duty * (_voltage / _resistor_ohm) * _delta_t;
        if(_current_time > 1.0f){
            _current_ave = _current_1sec_sum;
            _current_1sec_sum = 0.0f;
            _current_time = 0.0f;
        }
        _current_time += _delta_t;
    }


    void Heater::eval(uint32_t num){
        float time = 0.0f;

        PRINTF_ASYNC("---------------------\n");
        PRINTF_ASYNC("time[sec], temp[degC], duty, duty_p, duty_i\n");

        for (uint16_t i = 0; i < num; i++) {            
            uint64_t start_usec = hal::getElapsedUsec();
            float duty_p = _pidf.getPVal();
    		float duty_i = _pidf.getIVal();
            PRINTF_ASYNC("%.3f, %f, %f, %f, %f\n", time, _temp, _duty, duty_p, duty_i);
            time += 0.25f;
            while(1){
                uint64_t end_usec = hal::getElapsedUsec();
                if(end_usec - start_usec > 250000) break;
            }
        }
    }


    void Heater::debug(){
        PRINTF_ASYNC("  ------------------------\n");
        PRINTF_ASYNC("  p              : %f\n", _p);
        PRINTF_ASYNC("  i              : %f\n", _i);
        PRINTF_ASYNC("  i_limit        : %f\n", _i_limit);
        PRINTF_ASYNC("  limit          : %f\n", _limit);
        PRINTF_ASYNC("  ------------------------\n");        
        PRINTF_ASYNC("  target_temp    : %f\n", _target_temp);
        PRINTF_ASYNC("  temp           : %f\n", _temp);
        PRINTF_ASYNC("  ------------------------\n");        
        PRINTF_ASYNC("  duty           : %f\n", _duty);
        PRINTF_ASYNC("  duty_p         : %f\n", _pidf.getPVal());
        PRINTF_ASYNC("  duty_i         : %f\n", _pidf.getIVal());
        PRINTF_ASYNC("  ------------------------\n");
        PRINTF_ASYNC("  current_ave    : %f\n", _current_ave);
    }

    int usrcmd_heater(int argc, char **argv){

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            Heater::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "eval") == 0){
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }

            std::string num_str(argv[2]);
            int32_t num = std::stoi(num_str);
            Heater::getInstance().eval(num);
            return 0;
        }


        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    };



}
