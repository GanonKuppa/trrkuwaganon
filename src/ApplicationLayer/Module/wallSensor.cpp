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

// Module
#include "parameterManager.h"

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
        _on_wall_center_time(0.0f),
        _ahead_l_time_buf(),
        _ahead_r_time_buf(),
        _left_time_buf(),
        _right_time_buf(),
        _ahead_l_buf(),
        _ahead_r_buf(),
        _left_buf(),
        _right_buf()
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

    
    
    void WallSensor::update2(){
        if(!_enable){
            _turnLed(0, 0, 0, 0);
            return;
        }          
        _updateOffVal(1, 0, 1, 0);

        _turnLed(1, 0, 0, 0);
        //hal::waitusec(LED_ON_USEC);
        {
            uint8_t i = 0;
            float start_time = hal::hrtGetElapsedUsec();            
            while(1){
                float elapsed_time = hal::hrtGetElapsedUsec(start_time);
                _ahead_l_buf[i] = hal::startAD1();
                _ahead_l_time_buf[i] = elapsed_time;
                i++;
                if(elapsed_time > LED_ON_USEC || i > 80) break;
            }
        }
        _updateOnVal(1, 0, 0, 0);

        _turnLed(0, 0, 1, 0);
        //hal::waitusec(LED_ON_USEC);
        {
            uint8_t i = 0;
            float start_time = hal::hrtGetElapsedUsec();
            while(1){
                float elapsed_time = hal::hrtGetElapsedUsec(start_time);
                _right_buf[i] = hal::startAD3();
                _right_time_buf[i] = elapsed_time;
                i++;
                if(elapsed_time > LED_ON_USEC || i > 80) break;
            }
        }
                
        _updateOnVal(0, 0, 1, 0);

        _turnLed(0, 0, 0, 0);
    }

    void WallSensor::update3(){
        if(!_enable){
            _turnLed(0, 0, 0, 0);
            return;
        }  
        _updateOffVal(0, 1, 0, 1);


        _turnLed(0, 1, 0, 0);
        //hal::waitusec(LED_ON_USEC);
        {
            uint8_t i = 0;
            float start_time = hal::hrtGetElapsedUsec();            
            while(1){
                float elapsed_time = hal::hrtGetElapsedUsec(start_time);
                _left_buf[i] = hal::startAD2();
                _left_time_buf[i] = elapsed_time;
                i++;
                if(elapsed_time > LED_ON_USEC || i > 80) break;
            }
        }
        
        _updateOnVal(0, 1, 0, 0);

        _turnLed(0, 0, 0, 1);
        //hal::waitusec(LED_ON_USEC);
        {
            uint8_t i = 0;
            float start_time = hal::hrtGetElapsedUsec();            
            while(1){
                float elapsed_time = hal::hrtGetElapsedUsec(start_time);
                _ahead_r_buf[i] = hal::startAD4();
                _ahead_r_time_buf[i] = elapsed_time;
                i++;
                if(elapsed_time > LED_ON_USEC || i > 80) break;
            }
        }
        _updateOnVal(0, 0, 0, 1);

        _turnLed(0, 0, 0, 0);
        
        _modulateVal();

        _dist_al = _aheadDistL((float)_ahead_l_q.at(0));
        _dist_ar = _aheadDistR((float)_ahead_r_q.at(0));
        _dist_a = _aheadDist(_dist_al, _dist_ar);
        _dist_l = _distL((float)_left_q.at(0));
        _dist_r = _distR((float)_right_q.at(0));

        ParameterManager& pm = ParameterManager::getInstance();
        _is_ahead_l = (_dist_al <= pm.wall_al_thr); // 0.125f 
        _is_ahead_r = (_dist_ar <= pm.wall_ar_thr); // 0.125f
        _is_ahead = (_is_ahead_l || _is_ahead_r);
        _is_left = (_dist_l <= pm.wall_l_thr); // 0.0575f
        _is_right = (_dist_r <= pm.wall_r_thr); // 0.0575f
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
        //20211123 @自宅
        1486  0.045
        175	  0.09
        43	  0.135
        */
        constexpr float a =  -0.0250449f;
        constexpr float b =   0.2254917f;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;
    }

    float  WallSensor::_aheadDistR(float ad){
        /*
        //20211123 @自宅
        556	0.045
        66	0.09
        13	0.135

        */
        constexpr float a = -0.0238185f;
        constexpr float b = 0.1938118;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;
    }

    float  WallSensor::_aheadDist(float dist_al, float dist_ar){
        if(dist_al < 0.1f && dist_ar < 0.1f){
            return (dist_al + dist_ar) * 0.5f;
        }
        else{
            return std::max(dist_al, dist_ar);
        }
    }


    float WallSensor::_distL(float ad){
        /*
        //20211123@自宅
        105	 0.045
        1363 0.022
        21	 0.068
        */
        constexpr float a = -0.0108347f;
        constexpr float b = 0.0988697f;
        float x = std::max(ad, 1.0f);
        return a * std::log(x) + b;

    }

    float  WallSensor::_distR(float ad){
        /*
        //20211123@自宅
        193	 0.045
        2664 0.022
        40	 0.068
        */
        constexpr float a = -0.0107316f;
        constexpr float b = 0.1052369f;
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
    }

    void WallSensor::eval() {
        PRINTF_ASYNC("-------------------------------------------\n");
        PRINTF_ASYNC("t_la, la, t_l, l, t_r, r, t_ra, ra\n");
        for(uint8_t i=0;i<80;i++){
            PRINTF_ASYNC("%f, %d, %f, %d, %f, %d, %f, %d\n",
            _ahead_l_time_buf[i],
            _ahead_l_buf[i],
            _left_time_buf[i],
            _left_buf[i],
            _right_time_buf[i],
            _right_buf[i],
            _ahead_r_time_buf[i],
            _ahead_r_buf[i]
            );
        }

    }


    int usrcmd_wallSensor(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            WallSensor::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "eval") == 0) {
            WallSensor::getInstance().eval();
            return 0;
        }


        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    }
}



