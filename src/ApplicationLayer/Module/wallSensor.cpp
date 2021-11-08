#include "wallSensor.h"

#include <algorithm>
#include <cmath>

// Lib
#include "debugLog.h"
#include "ntlibc.h"

// Hal
#include "hal_ad.h"
#include "hal_gpio.h"
#include "hal_timer.h"

// Msg
#include "wallSensorMsg.h"
#include "msgBroker.h"


namespace module {
    
    WallSensor::WallSensor() :
        _enable(true),
        _ahead_l_on(0),
        _ahead_r_on(0),
        _left_on(0),
        _right_on(0),
        _ahead_l_off(0),
        _ahead_r_off(0),
        _left_off(0),
        _right_off(0),
        _dist_al(0.0f),
        _dist_ar(0.0f),
        _dist_a(0.0f),
        _dist_l(0.0f),
        _dist_r(0.0f),
        _is_ahead_l(false),
        _is_ahead_r(false),
        _is_ahead(false),
        _is_left(false),
        _is_right(false),
        _is_left_ctrl(false),
        _is_right_ctrl(false),
        _is_contact_wall(false),
        _is_on_wall_center(false),
        _contact_wall_time(0.0f),
        _on_wall_ahead_time(0.0f),
        _on_wall_center_time(0.0f)
    {
        setModuleName("WallSensor");
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

    
    
    void WallSensor::update0(){
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

    void WallSensor::update1(){
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

        _dist_al = _aheadDistL((float)_ahead_l_q.at(0));
        _dist_ar = _aheadDistR((float)_ahead_r_q.at(0));
        _dist_a = _aheadDist(_dist_al, _dist_ar);
        _dist_l = _distL((float)_left_q.at(0));
        _dist_r = _distR((float)_right_q.at(0));

        _is_ahead_l = (_dist_al <= 0.115f); 
        _is_ahead_r = (_dist_ar <= 0.09f);
        _is_ahead = (_is_ahead_l || _is_ahead_r);
        _is_left = (_dist_l <= 0.055f);
        _is_right = (_dist_r <= 0.055f);
        _is_left_ctrl = _is_left;
        _is_right_ctrl = _is_right;
        _is_contact_wall = (_dist_al < 0.04) && (_dist_ar < 0.04);
        _is_on_wall_center = (std::fabs(_dist_r - 0.045f) < 0.005) || (std::fabs(_dist_l - 0.045f) < 0.005);
        
        if(_is_contact_wall){
            _contact_wall_time += _delta_t;
        }
        else{
            _contact_wall_time = 0.0f;
        }

        if(_is_ahead){
            _on_wall_ahead_time += _delta_t;
        }
        else{
            _on_wall_ahead_time = 0.0f;
        }

        if(_is_on_wall_center){
            _on_wall_center_time += _delta_t;
        }
        else{
            _on_wall_center_time = 0.0f;
        }

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
    }


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
        msg.dist_al = _dist_al;
        msg.dist_ar = _dist_ar;
        msg.dist_a = _dist_a;
        msg.dist_l = _dist_l;
        msg.dist_r = _dist_r;

        msg.is_ahead_l = _is_ahead_l;
        msg.is_ahead_r = _is_ahead_r;
        msg.is_ahead = _is_ahead;
        msg.is_left = _is_left;
        msg.is_right = _is_right;    
        msg.is_left_ctrl = _is_left_ctrl;
        msg.is_right_ctrl = _is_right_ctrl;
        msg.is_on_wall_center = _is_on_wall_center;

        msg.contact_wall_time = _contact_wall_time;
        msg.on_wall_ahead_time = _on_wall_ahead_time;
        msg.on_wall_center_time = _on_wall_center_time;
        

        publishMsg(msg_id::WALL_SENSOR, &msg);
    }

    float  WallSensor::_aheadDistL(float ad){
        /*
        935	0.04
        460	0.05
        235	0.06
        120	0.07
        64	0.08
        38	0.09
        21	0.1
        13	0.11
        10	0.12
        */
        constexpr float a = -0.01705773f;
        constexpr float b = 0.15377920f;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;
    }

    float  WallSensor::_aheadDistR(float ad){
        /*
        255	0.04
        105	0.05
        38	0.06
        18	0.07
        8	0.08
        7	0.09
        */
        constexpr float a = -0.01282417f;
        constexpr float b = 0.10934711f;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;
    }

    float  WallSensor::_aheadDist(float dist_al, float dist_ar){
        if(dist_al < 0.08f && dist_ar < 0.08f){
            return (dist_al + dist_ar) * 0.5f;
        }
        else{
            return std::max(dist_al, dist_ar);
        }
    }


    float WallSensor::_distL(float ad){
        /*
        564	0.02
        97	0.03
        38	0.04
        16	0.05
        7	0.06
        */
        constexpr float a = -0.0093275f;
        constexpr float b = 0.0770684f;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;

    }

    float  WallSensor::_distR(float ad){
        /*
        1237	0.02
        290	0.03
        76	0.04
        22	0.05
        */
        constexpr float a = -0.0074383f;
        constexpr float b = 0.0725859f;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;
    }



    void WallSensor::debug() {        
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  ON     : al, l, r, ar: %d, %d, %d, %d\n", _ahead_l_on, _left_on,_right_on, _ahead_r_on);
        PRINTF_ASYNC("  OFF    : al, l, r, ar: %d, %d, %d, %d\n", _ahead_l_off, _left_off, _right_off, _ahead_r_off);
        PRINTF_ASYNC("  MOD    : al, l, r, ar: %d, %d, %d, %d\n", _ahead_l_q.at(0), _left_q.at(0), _right_q.at(0), _ahead_r_q.at(0));
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  dist   : al, l, r, ar: %f, %f, %f, %f\n", _dist_al, _dist_l, _dist_r, _dist_ar);
        PRINTF_ASYNC("  dist_a : %f\n", _dist_a);
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  is_ahead_l          : %d\n", _is_ahead_l);
        PRINTF_ASYNC("  is_ahead_r          : %d\n", _is_ahead_r);
        PRINTF_ASYNC("  is_ahead            : %d\n", _is_ahead);
        PRINTF_ASYNC("  is_left             : %d\n", _is_left);
        PRINTF_ASYNC("  is_right            : %d\n", _is_right);
        PRINTF_ASYNC("  is_left_ctrl        : %d\n", _is_left_ctrl);
        PRINTF_ASYNC("  is_right_ctrl       : %d\n", _is_right_ctrl);
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  is_contact_wall     : %d\n", _is_contact_wall);
        PRINTF_ASYNC("  is_on_wall_center   : %d\n", _is_on_wall_center);
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  contact_wall_time   : %f\n", _contact_wall_time);
        PRINTF_ASYNC("  on_wall_ahead_time  : %f\n", _on_wall_ahead_time);
        PRINTF_ASYNC("  on_wall_center_time : %f\n", _on_wall_center_time);
    };

    int usrcmd_wallSensor(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            WallSensor::getInstance().debug();
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    };
}



