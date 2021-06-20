#include "hal_ad.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "debugLog.h"

#include "wallSensor.h"
#include "wallSensorMsg.h"
#include "msgBroker.h"


namespace module {
    
    WallSensor::WallSensor() {

        _enable = true;
        _ahead_l_on = 0;
        _ahead_r_on = 0;
        _left_on = 0;
        _right_on = 0;

        _ahead_l_off = 0;
        _ahead_r_off = 0;
        _left_off = 0;
        _right_off = 0;


        for (uint8_t i = 0; i < BUFF_SIZE; i++) {
            _ahead_l_q.push_front(0);
            _ahead_r_q.push_front(0);
            _right_q.push_front(0);
            _left_q.push_front(0);
        }
    }
    
    
    void WallSensor::setEnable(bool en){
        _enable = en;
    }

    
    
    void WallSensor::update1(){
        if(!_enable){
            _turnLed(0, 0, 0, 0);
            return;
        }          
        _updateOffVal(1, 0, 1, 0);

        _turnLed(1, 0, 0, 0);
        hal::waitusec_sub(LED_ON_USEC);
        _updateOnVal(1, 0, 0, 0);

        _turnLed(0, 0, 1, 0);
        hal::waitusec_sub(LED_ON_USEC);
        _updateOnVal(0, 0, 1, 0);

        _turnLed(0, 0, 0, 0);
    }

    void WallSensor::update2(){
        if(!_enable){
            _turnLed(0, 0, 0, 0);
            return;
        }  
        _updateOffVal(0, 1, 0, 1);


        _turnLed(0, 1, 0, 0);
        hal::waitusec_sub(LED_ON_USEC);
        _updateOnVal(0, 1, 0, 0);

        _turnLed(0, 0, 0, 1);
        hal::waitusec_sub(LED_ON_USEC);
        _updateOnVal(0, 0, 0, 1);

        _turnLed(0, 0, 0, 0);
        
        _modulateVal();
        _publish();

    }



    void WallSensor::_updateOffVal(bool sled1, bool sled2, bool sled3, bool sled4){
        if(sled1) {
            _ahead_l_off = hal::startAD1();
            _ahead_l_off = hal::startAD1();
        }
        if(sled2) {
            _left_off = hal::startAD2();
            _left_off = hal::startAD2();
        }
        if(sled3) {
            _right_off = hal::startAD3();
            _right_off = hal::startAD3();
        }
        if(sled4) {
            _ahead_r_off = hal::startAD4();
            _ahead_r_off= hal::startAD4();
        }
    }

    void WallSensor::_updateOnVal(bool sled1, bool sled2, bool sled3, bool sled4) {
        if(sled1) {
            _ahead_l_on = hal::startAD1();
            _ahead_l_on = hal::startAD1();
        }
        if(sled2) {
            _left_on = hal::startAD2();
            _left_on = hal::startAD2();
        }
        if(sled3) {
            _right_on = hal::startAD3();
            _right_on = hal::startAD3();
        }
        if(sled4) {
            _ahead_r_on = hal::startAD4();
            _ahead_r_on = hal::startAD4();
        }
    }

    void WallSensor::_turnLed(bool sled1, bool sled2, bool sled3, bool sled4) {     
        if(sled1){
            hal::setDout3(1);
            hal::setDout4(1);
            
            hal::setDout5(1);                       
        } 
                            
        if(sled2){
        	hal::setDout3(1);
        	hal::setDout4(0);
        	
            hal::setDout5(1);            
        }

        if(sled3){
        	hal::setDout3(0);
        	hal::setDout4(0);
            
            hal::setDout5(1);            
        } 
                            
        if(sled4){
        	hal::setDout3(0);
        	hal::setDout4(1);
        	
            hal::setDout5(1);            
        }
        if(!sled1 && !sled2 && !sled3 && !sled4){
            hal::setDout5(0);
        }       
    };


    void WallSensor::_modulateVal() {
        int16_t ahl_mod = _ahead_l_on - _ahead_l_off;
        int16_t ahr_mod = _ahead_r_on - _ahead_r_off;
        int16_t l_mod = _left_on - _left_off;
        int16_t r_mod = _right_on - _right_off;

        _ahead_l_q.push_front(ahl_mod);
        _ahead_r_q.push_front(ahr_mod);
        _left_q.push_front(l_mod);
        _right_q.push_front(r_mod);

        _ahead_l_q.pop_back();
        _ahead_r_q.pop_back();
        _left_q.pop_back();
        _right_q.pop_back();
    }

    void WallSensor::_publish() {        
        WallSensorMsg msg;
        msg.ahead_l = _ahead_l_q.at(0);
        msg.ahead_r = _ahead_r_q.at(0);
        msg.left = _left_q.at(0);
        msg.right = _right_q.at(0);
        publishMsg(msg_id::WALL_SENSOR, &msg);
    };

    void WallSensor::debug() {        
        PRINTF_SYNC("=============================\n");
        PRINTF_SYNC("ON : al, l, r, ar: %d, %d, %d, %d\n", _ahead_l_on, _left_on,_right_on, _ahead_r_on);
        PRINTF_SYNC("OFF: al, l, r, ar: %d, %d, %d, %d\n", _ahead_l_off, _left_off, _right_off, _ahead_r_off);
        PRINTF_SYNC("MOD: al, l, r, ar: %d, %d, %d, %d\n", _ahead_l_q.at(0), _left_q.at(0), _right_q.at(0), _ahead_r_q.at(0));
        PRINTF_SYNC("=============================\n");
    };
}



