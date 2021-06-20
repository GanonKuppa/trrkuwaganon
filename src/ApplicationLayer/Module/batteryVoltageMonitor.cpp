
#include "hal_ad.h"
#include "debugLog.h"

#include "batteryVoltageMonitor.h"
#include "batteryVoltageMsg.h"
#include "msgBroker.h"


namespace module {
    BatteryVoltageMonitor::BatteryVoltageMonitor(){
    	_count = 0;
    	_voltage = 4.2f;
    	_voltage_ave = 4.2f;

        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            _buff.push_front(4.2);
        }

    }

    void BatteryVoltageMonitor::update(){
        int16_t ad = hal::startAD0();
        ad = hal::startAD0(); // 電荷を抜くために複数回AD変換
        _voltage = _ad2Voltage(ad);

        uint16_t count_500msec = 0.5f/_delta_t;
        if(_count == 0) {
            _buff.push_front(ad);
            _buff.pop_back();

            uint32_t sum = 0;
            for(auto itr = _buff.begin(); itr != _buff.end(); ++itr) {
                sum += *itr;
            }

            _voltage_ave = V_COEF * REG_VOLTAGE * (float(sum) / float(BUFF_SIZE)) / float(AD_RESOLUTION -1);
        }

        _count ++;
        if(_count > count_500msec) _count = 0;        
        _lowVoltageCheck();
        _publish();

    }

    float BatteryVoltageMonitor::_ad2Voltage(int16_t ad) {
        return V_COEF * REG_VOLTAGE * (float(ad) / float(AD_RESOLUTION -1));
    }

    void BatteryVoltageMonitor::_lowVoltageCheck() {
        if(_voltage_ave < ALERT_VOL) {
             _is_low_voltage = true;
        }
        else{
            _is_low_voltage = false;
        }
    }

    void BatteryVoltageMonitor::debug() {
        PRINTF_SYNC("================\n");
        PRINTF_SYNC("now:%f \n", _voltage);
        PRINTF_SYNC("ave %f \n", _voltage_ave);

    }

    void BatteryVoltageMonitor::_publish(){
        BatteryVoltageMsg msg;
        msg.voltage = _voltage;
        msg.voltage_ave = _voltage_ave;
        msg.is_low_voltage = _is_low_voltage;
        publishMsg(msg_id::BATTERY_VOLTAGE, &msg);
    }


}
