#include "powerTransmission.h"

#include <cmath>
#include <algorithm>
#include <cfloat>
#include <string>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_pwm.h"

// Module
#include "parameterManager.h"
#include "navigator.h"

// Msg
#include "msgBroker.h"
#include "actuatorOutputMsg.h"
#include "batteryInfoMsg.h"
#include "navStateMsg.h"
#include "wheelOdometryMsg.h"

namespace module {

    PowerTransmission::PowerTransmission() :
    _duty_r(0.0f),
    _duty_l(0.0f),
    _duty_r_normed(0.0f),
    _duty_l_normed(0.0f),
    _voltage(4.2f)
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

        NavStateMsg nav_msg;
        copyMsg(msg_id::NAV_STATE, &nav_msg);

        float duty_limit = ParameterManager::getInstance().duty_limit;

        if(out_msg.ctrl_mode == ECtrlMode::DIRECT_DUTY_SET || nav_msg.mode == ENavMode::DEBUG){
            // do nothing
        }
        else if(out_msg.ctrl_mode == ECtrlMode::PSEUDO_DIAL){
            float duty_l = std::clamp<float>(out_msg.duty_l, -duty_limit, duty_limit);
            float duty_r = std::clamp<float>(out_msg.duty_r, -duty_limit, duty_limit);
            setMaxVoltageDutyL(duty_l);
            setMaxVoltageDutyR(duty_r);
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
            float duty_l = std::clamp<float>(out_msg.duty_l, -duty_limit, duty_limit);
            float duty_r = std::clamp<float>(out_msg.duty_r, -duty_limit, duty_limit);
            setNormalizedDutyL(duty_l);
            setNormalizedDutyR(duty_r);
        }
        else{
            float duty_l = std::clamp<float>(out_msg.duty_l, -duty_limit, duty_limit);
            float duty_r = std::clamp<float>(out_msg.duty_r, -duty_limit, duty_limit);
            setNormalizedDutyL(duty_l);
            setNormalizedDutyR(duty_r);
        }
    }

    void PowerTransmission::debug(){

        NavStateMsg nav_msg;
        copyMsg(msg_id::NAV_STATE, &nav_msg);

        PRINTF_ASYNC("  duty_r        : %f\n", _duty_r);
        PRINTF_ASYNC("  duty_l        : %f\n", _duty_l);
        PRINTF_ASYNC("  duty_r_normed : %f\n", _duty_r_normed);
        PRINTF_ASYNC("  duty_l_normed : %f\n", _duty_l_normed);
        PRINTF_ASYNC("  voltage       : %f\n", _voltage);
        PRINTF_ASYNC("  nav_state     : %d\n", (uint8_t)nav_msg.mode);
    }

    void PowerTransmission::selftestDuty(){
        Navigator::getInstance().setNavMode(ENavMode::DEBUG);
        hal::waitmsec(10);
        PRINTF_ASYNC("---------------\n");
        PRINTF_ASYNC("i, j, duty, vol_l, v_l, v_l_ave, rpm_l, rpm_l_ave, vol_r, v_r, v_r_ave, rpm_r, rpm_r_ave\n");
        for(int i=-20;i<20;i++){
            float duty = (float)i*0.01;
            setDutyR(duty);
            setDutyL(duty);
            
            hal::waitmsec(500);

            for(int j=0;j<100;j++){
                float vol_l = _voltage * _duty_l;
                float vol_r = _voltage * _duty_r;
                WheelOdometryMsg wodo_msg;
                copyMsg(msg_id::WHEEL_ODOMETRY, &wodo_msg);
                PRINTF_ASYNC("%d, %d, %f,"
                            "%f, %f, %f, %f, %f, " 
                            "%f, %f, %f, %f, %f\n",                            
                            i ,j ,duty, 
                            vol_l, wodo_msg.v_l, wodo_msg.v_l_ave, wodo_msg.rpm_l, wodo_msg.rpm_l_ave,
                            vol_r, wodo_msg.v_r, wodo_msg.v_r_ave, wodo_msg.rpm_r, wodo_msg.rpm_r_ave);
                hal::waitmsec(10);
            }
            setDutyR(0.0f);
            setDutyL(0.0f);
            hal::waitmsec(500);
        }
          

    }

    void PowerTransmission::selftestNormalizedDuty(){
        Navigator::getInstance().setNavMode(ENavMode::DEBUG);
        hal::waitmsec(10);
        PRINTF_ASYNC("---------------\n");
        PRINTF_ASYNC("i, j, normalized_duty, vol_l, v_l, v_l_ave, rpm_l, rpm_l_ave, vol_r, v_r, v_r_ave, rpm_r, rpm_r_ave\n");
        for(int i=-20;i<20;i++){
            float duty = (float)i*0.01;
            setNormalizedDutyR(duty);
            setNormalizedDutyL(duty);
            
            hal::waitmsec(500);

            for(int j=0;j<100;j++){
                float vol_l = _voltage * _duty_l;
                float vol_r = _voltage * _duty_r;
                WheelOdometryMsg wodo_msg;
                copyMsg(msg_id::WHEEL_ODOMETRY, &wodo_msg);
                PRINTF_ASYNC("%d, %d, %f,"
                            "%f, %f, %f, %f, %f, " 
                            "%f, %f, %f, %f, %f\n",                            
                            i ,j ,duty, 
                            vol_l, wodo_msg.v_l, wodo_msg.v_l_ave, wodo_msg.rpm_l, wodo_msg.rpm_l_ave,
                            vol_r, wodo_msg.v_r, wodo_msg.v_r_ave, wodo_msg.rpm_r, wodo_msg.rpm_r_ave);
                hal::waitmsec(10);
            }
            setNormalizedDutyR(0.0f);
            setNormalizedDutyL(0.0f);
            hal::waitmsec(500);
        }
          

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

    void PowerTransmission::setNormalizedDutyR(float duty){
        ParameterManager& pm = ParameterManager::getInstance();
        
        if(duty > 0.0f){
            _duty_r_normed = duty;
            float max_voltage_duty = duty * pm.duty_coef_right_p + pm.duty_offset_right_p;
            setMaxVoltageDutyR(max_voltage_duty);
        }
        else if(duty < 0.0f){
            _duty_r_normed = duty;
            float max_voltage_duty = duty * pm.duty_coef_right_m - pm.duty_offset_right_m;
            setMaxVoltageDutyR(max_voltage_duty);
        }else{
            _duty_r_normed = 0.0f;
            setDutyR(0.0f);
        }        
    }

    void PowerTransmission::setNormalizedDutyL(float duty){
        ParameterManager& pm = ParameterManager::getInstance();

    	if(duty > 0.0f){
            _duty_l_normed = duty;
            float max_voltage_duty = duty * pm.duty_coef_left_p + pm.duty_offset_left_p;
            if(std::fabs(max_voltage_duty) < pm.duty_offset_left_p){
                max_voltage_duty = 0.0f;
            }
            setMaxVoltageDutyL(max_voltage_duty);
        }
        else if(duty < 0.0f){
            _duty_l_normed = duty;
            float max_voltage_duty = duty * pm.duty_coef_left_m - pm.duty_offset_left_m;
            if(std::fabs(max_voltage_duty) < pm.duty_offset_left_m){
                max_voltage_duty = 0.0f;
            }            
            setMaxVoltageDutyL(max_voltage_duty);
        }else{
            _duty_l_normed = 0.0f;
            setDutyL(0.0f);
        }        
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
            Navigator::getInstance().setNavMode(ENavMode::DEBUG);
            hal::waitmsec(10);
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
            Navigator::getInstance().setNavMode(ENavMode::DEBUG);
            hal::waitmsec(10);
            float duty_val = std::stof(duty_val_str);
            PowerTransmission::getInstance().setDutyL(duty_val);

            return 0;
        }

        if (ntlibc_strcmp(argv[1], "selftest_duty") == 0) {
            PowerTransmission::getInstance().selftestDuty();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "selftest_normalized_duty") == 0) {
            PowerTransmission::getInstance().selftestNormalizedDuty();
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    };



}
