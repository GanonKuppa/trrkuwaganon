#include "heater.h"
#include "pidController.h"

#include "parameterManager.h"
#include "imuMsg.h"
#include "msgBroker.h"



namespace module{
    Heater::Heater() :
    _duty(0.0f)
    {
        setModuleName("Heater");
        ParameterManager& pm = ParameterManager::getInstance();
        _p = pm.heater_p;
        _i = pm.heater_i;
        _i_limit = pm.heater_i_limit;
        _limit = pm.heater_limit;
        _target_temp = pm.heater_target_temp;
        _pidf.set(0.0f, 0.0f, 0.0f, 0.0f);
    }

    float Heater::getDuty(){
        return _duty;
    }

    void Heater::setTargetTemp(float temp){
        _target_temp = temp;
        ParameterManager& pm = ParameterManager::getInstance();
        pm.write<float>("heater_target_temp", temp);
    }    

    void Heater::update0(){
    	ParameterManager& pm = ParameterManager::getInstance();
        
        ImuMsg imu_msg;
        copyMsg(msg_id::IMU, &imu_msg);
        _temp = imu_msg.temp;

        _p = pm.heater_p;
        _i = pm.heater_i;
        _i_limit = pm.heater_i_limit;
        _limit = pm.heater_limit;
        _target_temp = pm.heater_target_temp;
        _pidf.setSaturationEnable(true);
        _pidf.setSaturation(_limit);
        _pidf.setIntegralSaturationEnable(true);
        _pidf.setIntegralSaturation(_i_limit);
        _pidf.update(_target_temp, _temp);
        _duty = _pidf.getControlVal();

    }
    
}
