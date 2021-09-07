#include "powerTransmission.h"

#include <cmath>
#include <algorithm>
#include <cfloat>
#include <string>

// Lib
#include "ntlibc.h"

// Hal
#include "hal_pwm.h"

// Module
#include "parameterManager.h"

// Msg
#include "msgBroker.h"
#include "actuatorOutputMsg.h"
#include "batteryInfoMsg.h"
#include "navigator.h"

namespace module {

    PowerTransmission::PowerTransmission() :
    _duty_r(0.0f),
    _duty_l(0.0f)
    {
        setModuleName("PowerTransmission");
        setDutyL(0.0f);
        setDutyR(0.0f);
    }

    void PowerTransmission::update0(){
        ActuatorOutputMsg out_msg;
        copyMsg(msg_id::ACTUATOR_OUTPUT, &out_msg);

        BatteryInfoMsg bat_msg;
        copyMsg(msg_id::BATTERY_INFO, &bat_msg);
        _voltage = bat_msg.voltage;
        
        if(out_msg.ctrl_mode == ECtrlMode::PSEUDO_DIAL){
            setMaxVoltageDutyL(out_msg.duty_l);
            setMaxVoltageDutyR(out_msg.duty_r);
        }
        else if(out_msg.ctrl_mode == ECtrlMode::VEHICLE){
/*
            // duty飽和時には回転系制御を優先
            if(duty(0) > 1.0 || duty(1) > 1.0) {
                float duty_overflow = 0.0f;
                if(duty(0) > duty(1) ) {
                    duty_overflow = duty(0) - 1.0f;
                } else {
                    duty_overflow = duty(1) - 1.0f;
                }
                duty(0) -= duty_overflow;
                duty(1) -= duty_overflow;

            }
*/
            setMaxVoltageDutyL(out_msg.duty_l);
            setMaxVoltageDutyR(out_msg.duty_r);
        }
        else{
            setMaxVoltageDutyL(0.0f);
            setMaxVoltageDutyR(0.0f);
        }
    }

    void PowerTransmission::debug(){

    }


    void PowerTransmission::setDutyR(float duty){
        if( std::fabs(duty - _duty_r) < FLT_EPSILON &&
            _duty_r < 0.999f &&
            _duty_r > 0.0f) return;


        ParameterManager& pm = ParameterManager::getInstance();
        float abs_duty = std::clamp<float>(std::fabs(duty) , 0.0f, pm.duty_limit);

        if(std::isnan(duty) || std::isnan(abs_duty)) {
            _duty_r = 0.0f;
        } 
        else{
            if(duty > 0.0f){
                _duty_r = abs_duty;
            }
            else{
                _duty_r = - abs_duty;
            }
        }

        if ( std::fabs(_duty_r) < FLT_EPSILON) {
            hal::setDutyPWM1(1.0f);
            hal::setDutyPWM2(1.0f);
        } 
        else if ( _duty_r < 0.0f) {
            hal::setDutyPWM1(1.0f - abs_duty);
            hal::setDutyPWM2(1.0f);
        } 
        else {
            hal::setDutyPWM1(1.0f);
            hal::setDutyPWM2(1.0f - abs_duty);
        }        
    }

    void PowerTransmission::setDutyL(float duty){
        if( std::fabs(duty - _duty_l) < FLT_EPSILON &&
            _duty_l < 0.999f &&
            _duty_l > 0.0f) return;
        
        ParameterManager& pm = ParameterManager::getInstance();
        float abs_duty = std::clamp<float>(std::fabs(duty) , 0.0f, pm.duty_limit);

        if(std::isnan(duty) || std::isnan(abs_duty)) {
            _duty_l = 0.0f;
        }
        else{
            if(duty > 0.0f){
                _duty_l = abs_duty;
            }
            else{
                _duty_l = - abs_duty;
            }
        }

        if ( std::fabs(_duty_l) < FLT_EPSILON) {
            hal::setDutyPWM3(1.0f);
            hal::setDutyPWM4(1.0f);
        } 
        else if ( _duty_l > 0.0f) {
            hal::setDutyPWM3(1.0f - abs_duty);
            hal::setDutyPWM4(1.0f);
        } 
        else {
            hal::setDutyPWM3(1.0f);
            hal::setDutyPWM4(1.0f - abs_duty);

        }        
    }    

    void PowerTransmission::setMaxVoltageDutyR(float duty){
        float max_vol_duty = duty * MAX_VOLTAGE / _voltage;
        setDutyR(max_vol_duty);
    }

    void PowerTransmission::setMaxVoltageDutyL(float duty){
        float max_vol_duty = duty * MAX_VOLTAGE / _voltage;
        setDutyL(max_vol_duty);
    }



    int usrcmd_powerTransmission(int argc, char **argv){

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            PowerTransmission::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "duty_r") == 0){
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }

            std::string duty_val_str(argv[2]);
            float duty_val = std::stof(duty_val_str);
            PowerTransmission::getInstance().setDutyR(duty_val);
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "duty_l") == 0){
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            std::string duty_val_str(argv[2]);
            float duty_val = std::stof(duty_val_str);
            PowerTransmission::getInstance().setDutyL(duty_val);

            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    };



}
