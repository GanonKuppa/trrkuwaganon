
#include "hal_ad.h"
#include "debugLog.h"

#include "batteryMonitor.h"
#include "batteryInfoMsg.h"
#include "msgBroker.h"

#include "ntlibc.h"


namespace module {
    BatteryMonitor::BatteryMonitor(){
    	setModuleName("BatteryMonitor");
        
        _count = 0;
    	_voltage = 4.2f;
    	_voltage_ave = 4.2f;

        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            _buff.push_front(4.2);
        }

    }

    void BatteryMonitor::update0(){
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

    float BatteryMonitor::_ad2Voltage(int16_t ad) {
        return V_COEF * REG_VOLTAGE * (float(ad) / float(AD_RESOLUTION -1));
    }

    void BatteryMonitor::_lowVoltageCheck() {
        if(_voltage_ave < ALERT_VOL) {
             _is_low_voltage = true;
        }
        else{
            _is_low_voltage = false;
        }
    }

    void BatteryMonitor::debug() {
        PRINTF_ASYNC("  ================\n");
        PRINTF_ASYNC("  now:%f \n", _voltage);
        PRINTF_ASYNC("  ave %f \n", _voltage_ave);

    }

    void BatteryMonitor::_publish(){
        BatteryInfoMsg msg;
        msg.voltage = _voltage;
        msg.voltage_ave = _voltage_ave;
        msg.is_low_voltage = _is_low_voltage;
        publishMsg(msg_id::BATTERY_INFO, &msg);
    }

    int usrcmd_batteryMonitor(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            BatteryMonitor::getInstance().debug();
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    };


}
