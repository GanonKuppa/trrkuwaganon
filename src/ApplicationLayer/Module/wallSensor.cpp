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
#include "hal_timerInterrupt.h"
#include "hal_critical_section.h"

// Module
#include "parameterManager.h"

// Msg
#include "wallSensorMsg.h"
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"


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
        _is_corner_l(false),
        _is_corner_r(false),
        _is_diag_corner_l(false),
        _is_diag_corner_r(false),
        _contact_wall_time(0.0f),
        _on_wall_ahead_time(0.0f),
        _on_wall_ahead_l_time(0.0f),
        _on_wall_ahead_r_time(0.0f),
        _on_wall_center_time(0.0f),
        _not_corner_l_elapsed_time(0.0f),
        _not_corner_r_elapsed_time(0.0f),
        _emit_led_cycle_time_us()
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
        _turnLed(0, 0, 0, 0);
        if(!_enable){            
            return;
        }

        CtrlSetpointMsg setp_msg;
        copyMsg(msg_id::CTRL_SETPOINT, &setp_msg);

        if(setp_msg.turn_type == ETurnType::TURN_90 ||
          (setp_msg.traj_type == ETrajType::SPINTURN && setp_msg.yawrate * setp_msg.yawacc >= 0.0) //?????????????????????????????????????????????????????????
        ){
            return;
        }

        _updateOffVal(1, 1, 1, 1);
        _modulateVal();

        _dist_al = _aheadDistL((float)_ahead_l_q.at(0));
        _dist_ar = _aheadDistR((float)_ahead_r_q.at(0));
        _dist_a = _aheadDist(_dist_al, _dist_ar);
        _dist_l = _distL((float)_left_q.at(0));
        _dist_r = _distR((float)_right_q.at(0));

        ParameterManager& pm = ParameterManager::getInstance();
        float wall_center_r = pm.wall_center_r;
        float wall_center_l = pm.wall_center_l;


        _is_ahead_l = (_dist_al <= pm.wall_al_thr);
        _is_ahead_r = (_dist_ar <= pm.wall_ar_thr);
        _is_ahead = (_is_ahead_l && _is_ahead_r);
        _is_left = (_dist_l <= pm.wall_l_thr);
        _is_right = (_dist_r <= pm.wall_r_thr);
        _is_left_ctrl = _is_left;
        _is_right_ctrl = _is_right;
        _is_contact_wall = (_dist_al < 0.04f) && (_dist_ar < 0.04f);
        _is_on_wall_center = (std::fabs(_dist_r - wall_center_r) < 0.005f) || (std::fabs(_dist_l - wall_center_l) < 0.005f);
        _is_corner_l = _isCornerL();
        _is_corner_r = _isCornerR();
        _is_diag_corner_l = _isDiagCornerL();
        _is_diag_corner_r = _isDiagCornerR();


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

        if(_is_ahead_l){
            _on_wall_ahead_l_time += _delta_t;
        }
        else{
            _on_wall_ahead_l_time = 0.0f;
        }

        if(_is_ahead_r){
            _on_wall_ahead_r_time += _delta_t;
        }
        else{
            _on_wall_ahead_r_time = 0.0f;
        }

        if(_is_on_wall_center){
            _on_wall_center_time += _delta_t;
        }
        else{
            _on_wall_center_time = 0.0f;
        }

        if(!_is_corner_l){
            _not_corner_l_elapsed_time += _delta_t;
        }
        else{
            _not_corner_l_elapsed_time = 0.0f;
        }

        if(!_is_corner_r){
            _not_corner_r_elapsed_time += _delta_t;
        }
        else{
            _not_corner_r_elapsed_time = 0.0f;
        }

        _publish();

        hal::setInterruptPeriodTimerInterrupt1(LED_ON_SEC);
        hal::restartTimerInterrupt1();
    }    

    void WallSensor::emitLedTask(){
        static uint8_t loop_count = 0;

        float start_usec = hal::hrtGetElapsedUsec();
        if(!_enable){            
            _turnLed(0, 0, 0, 0);
            hal::stopTimerInterrupt1();
            loop_count = 0;
            return;
        }

        if(loop_count % 8 == 0){                        
            _turnLed(1,0,0,0);
             _emit_led_cycle_time_us[0] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 1){
            _updateOnVal(1,0,0,0);
            _turnLed(0,0,0,0);
            _emit_led_cycle_time_us[1] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 2){
            _turnLed(0,0,1,0);
            _emit_led_cycle_time_us[2] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 3){
            _updateOnVal(0,0,1,0);
            _turnLed(0,0,0,0);
            _emit_led_cycle_time_us[3] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 4){
            _turnLed(0,1,0,0);
            _emit_led_cycle_time_us[4] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 5){
            _updateOnVal(0,1,0,0);
            _turnLed(0,0,0,0);
            _emit_led_cycle_time_us[5] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 6){
            _turnLed(0,0,0,1);
            _emit_led_cycle_time_us[6] = hal::hrtGetElapsedUsec(start_usec);
        }
        else if(loop_count % 8 == 7){
            _updateOnVal(0,0,0,1);
            _turnLed(0,0,0,0);                        
            _emit_led_cycle_time_us[7] = hal::hrtGetElapsedUsec(start_usec);            
            hal::stopTimerInterrupt1();
        }
        else{
            _turnLed(0,0,0,0);            
            hal::stopTimerInterrupt1();
        }

        loop_count ++;
    }


    void WallSensor::_updateOffVal(bool sled1, bool sled2, bool sled3, bool sled4){
        uint16_t ad_array[10];
        if(sled1) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD1();
            }
            _ahead_l_off = _trimAverage(ad_array, 10, 4);
        }
        if(sled2) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD2();
            }
            _left_off = _trimAverage(ad_array, 10, 4);
        }
        if(sled3) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD3();
            }
            _right_off = _trimAverage(ad_array, 10, 4);
        }
        if(sled4) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD4();
            }
            _ahead_r_off = _trimAverage(ad_array, 10, 4);
        }
    }

    void WallSensor::_updateOnVal(bool sled1, bool sled2, bool sled3, bool sled4) {
        uint16_t ad_array[10];
        if(sled1) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD1();
            }
            _ahead_l_on = _trimAverage(ad_array, 10, 4);
        }
        if(sled2) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD2();
            }
            _left_on = _trimAverage(ad_array, 10, 4);
        }
        if(sled3) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD3();
            }
            _right_on = _trimAverage(ad_array, 10, 4);
        }
        if(sled4) {
            for(uint8_t i=0;i<10;i++){
                ad_array[i] = hal::startAD4();
            }
            _ahead_r_on = _trimAverage(ad_array, 10, 4);
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
        msg.left_max_in_buff = _leftMax();
        msg.right_max_in_buff = _rightMax();

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
        msg.is_corner_l = _is_corner_l;
        msg.is_corner_r = _is_corner_r;
        msg.is_diag_corner_l = _is_diag_corner_l;
        msg.is_diag_corner_r = _is_diag_corner_r;

        msg.contact_wall_time = _contact_wall_time;
        msg.on_wall_ahead_time = _on_wall_ahead_time;
        msg.on_wall_ahead_l_time = _on_wall_ahead_r_time;
        msg.on_wall_ahead_r_time = _on_wall_ahead_l_time;
        msg.on_wall_center_time = _on_wall_center_time;
        msg.not_corner_l_elapsed_time = _not_corner_l_elapsed_time;
        msg.not_corner_r_elapsed_time = _not_corner_r_elapsed_time;

        publishMsg(msg_id::WALL_SENSOR, &msg);
    }

    float  WallSensor::_aheadDistL(float ad){
        ParameterManager &pm = ParameterManager::getInstance();
        float a = pm.al_dist_coef_a;
        float b = pm.al_dist_coef_b;
        float x = std::max(ad, 1.0f);
        return a * std::pow(ad, b);
    }

    float  WallSensor::_aheadDistR(float ad){
        ParameterManager &pm = ParameterManager::getInstance();
        float a = pm.ar_dist_coef_a;
        float b = pm.ar_dist_coef_b;
        float x = std::max(ad, 1.0f);
        return a * std::pow(ad, b);
    }

    float  WallSensor::_aheadDist(float dist_al, float dist_ar){
        if(dist_al < 0.1f && dist_ar < 0.1f){
            return (dist_al + dist_ar) * 0.5f;
        }
        else{
            return std::max(dist_al, dist_ar);
        }
    }

    bool WallSensor::_isCornerL() {
        ParameterManager& pm = ParameterManager::getInstance();
        int16_t th_on = pm.wall_corner_threshold_on_l;
        int16_t th_off = pm.wall_corner_threshold_off_l;

        if(_left_q.at(0) < th_off) {
            for(int i=1; i<BUFF_SIZE; i++) {
                if(_left_q.at(i)> th_on) {
                    return true;
                }
            }
        }
        return false;
    }

    bool WallSensor::_isCornerR() {
        ParameterManager& pm = ParameterManager::getInstance();
        int16_t th_on = pm.wall_corner_threshold_on_r;
        int16_t th_off = pm.wall_corner_threshold_off_r;        

        if(_right_q.at(0) < th_off) {
            for(int i=1; i<BUFF_SIZE; i++) {
                if(_right_q.at(i)> th_on) {
                    return true;
                }
            }
        }
        return false;
    }

    bool WallSensor::_isDiagCornerL() {
        ParameterManager& pm = ParameterManager::getInstance();
        int16_t th_on = pm.diag_corner_threshold_on_l;
        int16_t th_off = pm.diag_corner_threshold_off_l;

        if(_left_q.at(0) < th_off) {
            for(int i=1; i<BUFF_SIZE; i++) {
                if(_left_q.at(i)> th_on) {
                    return true;
                }
            }
        }
        return false;
    }

    bool WallSensor::_isDiagCornerR() {
        ParameterManager& pm = ParameterManager::getInstance();
        int16_t th_on = pm.diag_corner_threshold_on_r;
        int16_t th_off = pm.diag_corner_threshold_off_r;        

        if(_right_q.at(0) < th_off) {
            for(int i=1; i<BUFF_SIZE; i++) {
                if(_right_q.at(i)> th_on) {
                    return true;
                }
            }
        }
        return false;
    }



    float WallSensor::_distL(float ad){
        constexpr float a = 0.7591730f;
        constexpr float b = -0.4121754f;
        float x = std::max(ad, 1.0f);
        return a * std::pow(ad, b);

    }

    float  WallSensor::_distR(float ad){
        constexpr float a = 0.5269620f;
        constexpr float b = -0.3775151f;
        float x = std::max(ad, 1.0f);
        return a * std::pow(ad, b);
    }

    int16_t WallSensor::_leftMax(){
        int16_t max_val = 0;
        for(int i=0; i<BUFF_SIZE; i++) {
            if(_left_q.at(i) >= max_val) {
                max_val = _left_q.at(i);
            }
        }
        return max_val;
    }

    int16_t WallSensor::_rightMax(){
        int16_t max_val = 0;
        for(int i=0; i<BUFF_SIZE; i++) {
            if(_right_q.at(i) >= max_val) {
                max_val = _right_q.at(i);
            }
        }
        return max_val;
    }

    uint16_t WallSensor::_trimAverage(uint16_t ad_array[], uint16_t num, uint16_t trim_num){
        std::sort(ad_array, ad_array + (sizeof(ad_array)/sizeof(ad_array[0])));
        uint32_t sum = 0;
        for(uint16_t i=trim_num;i<num-trim_num;i++){
            sum += ad_array[i];
        }
        return sum / (num - trim_num * 2);
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
        PRINTF_ASYNC("  is_ahead_l                : %d\n", _is_ahead_l);
        PRINTF_ASYNC("  is_ahead_r                : %d\n", _is_ahead_r);
        PRINTF_ASYNC("  is_ahead                  : %d\n", _is_ahead);
        PRINTF_ASYNC("  is_left                   : %d\n", _is_left);
        PRINTF_ASYNC("  is_right                  : %d\n", _is_right);
        PRINTF_ASYNC("  is_left_ctrl              : %d\n", _is_left_ctrl);
        PRINTF_ASYNC("  is_right_ctrl             : %d\n", _is_right_ctrl);
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  is_contact_wall           : %d\n", _is_contact_wall);
        PRINTF_ASYNC("  is_on_wall_center         : %d\n", _is_on_wall_center);
        PRINTF_ASYNC("  is_corner_l               : %d\n", _is_corner_l);
        PRINTF_ASYNC("  is_corner_r               : %d\n", _is_corner_r);
        PRINTF_ASYNC("  =============================\n");
        PRINTF_ASYNC("  contact_wall_time         : %f\n", _contact_wall_time);
        PRINTF_ASYNC("  on_wall_ahead_time        : %f\n", _on_wall_ahead_time);
        PRINTF_ASYNC("  on_wall_center_time       : %f\n", _on_wall_center_time);
        PRINTF_ASYNC("  not_corner_l_elapsed_time : %f\n", _not_corner_l_elapsed_time);
        PRINTF_ASYNC("  not_corner_r_elapsed_time : %f\n", _not_corner_r_elapsed_time);
    }

    void WallSensor::eval() {
        hal::enterCriticalSection();
        _turnLed(0, 0, 0, 0);
        hal::hrtStopTimer();
        hal::hrtStartTimer();
        constexpr uint16_t BUF_SIZE = 70;
        float ahead_l_time_buf[BUF_SIZE];
        float ahead_r_time_buf[BUF_SIZE];
        float left_time_buf[BUF_SIZE];
        float right_time_buf[BUF_SIZE];
        int16_t ahead_l_buf[BUF_SIZE];
        int16_t ahead_r_buf[BUF_SIZE];
        int16_t left_buf[BUF_SIZE];
        int16_t right_buf[BUF_SIZE];
        
        hal::waitusec(150);
        float start_time = hal::hrtGetElapsedUsec();                
        for(uint8_t i=0;i<BUF_SIZE;i++){
            if(i == BUF_SIZE/2)_turnLed(1, 0, 0, 0);
            hal::startAD1();
            ahead_l_buf[i] = hal::startAD1();
            float elapsed_time = hal::hrtGetElapsedUsec(start_time);
            ahead_l_time_buf[i] = elapsed_time;
        }
        _turnLed(0, 0, 0, 0);
        hal::waitusec(150);

        start_time = hal::hrtGetElapsedUsec();
        for(uint8_t i=0;i<BUF_SIZE;i++){
            if(i == BUF_SIZE/2)_turnLed(0, 0, 1, 0);
            hal::startAD3();
            right_buf[i] = hal::startAD3();
            float elapsed_time = hal::hrtGetElapsedUsec(start_time);
            right_time_buf[i] = elapsed_time;
        }
        _turnLed(0, 0, 0, 0);
        hal::waitusec(150);

        start_time = hal::hrtGetElapsedUsec();            
        for(uint8_t i=0;i<BUF_SIZE;i++){
            if(i == BUF_SIZE/2)_turnLed(0, 1, 0, 0);
            hal::startAD2();
            left_buf[i] = hal::startAD2();
            float elapsed_time = hal::hrtGetElapsedUsec(start_time);
            left_time_buf[i] = elapsed_time;
        }
        _turnLed(0, 0, 0, 0);
        hal::waitusec(150);
        
        start_time = hal::hrtGetElapsedUsec();            
        for(uint8_t i=0;i<BUF_SIZE;i++){
            if(i == BUF_SIZE/2)_turnLed(0, 0, 0, 1);
            hal::startAD4();
            ahead_r_buf[i] = hal::startAD4();
            float elapsed_time = hal::hrtGetElapsedUsec(start_time);
            ahead_r_time_buf[i] = elapsed_time;
        }
        _turnLed(0, 0, 0, 0);
        
        hal::leaveCriticalSection();

        PRINTF_ASYNC("-------------------------------------------\n");
        PRINTF_ASYNC("t_al, al, t_l, l, t_r, r, t_ar, ar\n");
        for(uint8_t i=0;i<50;i++){
            PRINTF_ASYNC("%f, %d, %f, %d, %f, %d, %f, %d\n",
                ahead_l_time_buf[i], ahead_l_buf[i],
                left_time_buf[i], left_buf[i],
                right_time_buf[i], right_buf[i],
                ahead_r_time_buf[i], ahead_r_buf[i]
            );
        }
    }

    void WallSensor::printCycleTime(){
        PRINTF_ASYNC("    %s\n",getModuleName().c_str());
        PRINTF_ASYNC("      %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us]\n", _cycle_time_us[0], _cycle_time_us[1], _cycle_time_us[2], _cycle_time_us[3], _cycle_time_every_us, _cycle_time_in_main_loop_us);
        PRINTF_ASYNC("      -- emitLedTask() 0 to 7 --\n");        
        PRINTF_ASYNC("      %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us]\n", _emit_led_cycle_time_us[0], _emit_led_cycle_time_us[1], _emit_led_cycle_time_us[2], _emit_led_cycle_time_us[3]);
        PRINTF_ASYNC("      %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us]\n\n", _emit_led_cycle_time_us[4], _emit_led_cycle_time_us[5], _emit_led_cycle_time_us[6], _emit_led_cycle_time_us[7]);

    }

    int usrcmd_wallSensor(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status :\r\n");
            PRINTF_ASYNC("  eval   :\r\n");
            return 0;
        }

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



