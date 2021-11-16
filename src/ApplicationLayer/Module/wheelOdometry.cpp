#include "wheelOdometry.h"

#include <numeric>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_spi.h"

// Module
#include "parameterManager.h"
#include "powerTransmission.h"
#include "navigator.h"

// Msg
#include "msgBroker.h"
#include "wheelOdometryMsg.h"





namespace module {
    WheelOdometry::WheelOdometry() :
    _count_r(0),
    _count_l(0),
    _count_r_pre(0),
    _count_l_pre(0),
    _ang_r_deg(0.0f),
    _ang_l_deg(0.0f),
    _rpm_r(0.0f),
    _rpm_l(0.0f),
    _rpm_r_ave(0.0f),
    _rpm_l_ave(0.0f),
    _ang_r_cor(0.0f),
    _ang_l_cor(0.0f),
    _v_r(0.0f),
    _v_l(0.0f),
    _v_r_ave(0.0f),
    _v_l_ave(0.0f),
    _v(0.0f),    
    _yawrate_rad(0.0f),
    _yawrate_deg(0.0f)
    {
        setModuleName("WheelOdometry");
        for (uint8_t i = 0; i < AVERAGE_NUM; i++) {
            _v_r_list.push_front(0.0f);
            _v_l_list.push_front(0.0f);
        }

        _updateParam();        
    }


    void WheelOdometry::_readEncoder(){
        _count_r_pre = _count_r;
        _count_l_pre = _count_l;

        hal::useCS0SPI1();
        _count_r = hal::communicate16bitSPI1(0);
        _ang_r_deg = (float)_count_r / ENC_RES * 360.0f;
        if(ENC_R_DIR  < 0.0f) _ang_r_deg = 360.0f - _ang_r_deg;

        hal::useCS1SPI1();
        _count_l = hal::communicate16bitSPI1(0);
        _ang_l_deg = (float)_count_l / ENC_RES * 360.0f;
        if(ENC_L_DIR  < 0.0f) _ang_l_deg = 360.0f - _ang_l_deg;
    }

    void WheelOdometry::_updateParam(){
        ParameterManager& pm = ParameterManager::getInstance();
        _dia_tire = pm.dia_tire;
        _tread = pm.tread;
    }


    void WheelOdometry::update0(){
        _updateParam();
        _readEncoder();

        float _count_diff_r = (int32_t)(_count_r - _count_r_pre);
        float _count_diff_l = (int32_t)(_count_l - _count_l_pre);

        //オーバーフロー対策
        if (_count_diff_r >  ENC_RES/2){_count_diff_r -= ENC_RES;}
        if (_count_diff_r < -ENC_RES/2){_count_diff_r += ENC_RES;}
        if (_count_diff_l >  ENC_RES/2){_count_diff_l -= ENC_RES;}
        if (_count_diff_l < -ENC_RES/2){_count_diff_l += ENC_RES;}
            

        _rpm_r = (_count_diff_r / GEAR_RATIO / ENC_RES * ENC_R_DIR) / _delta_t * 60.0f;
        _rpm_l = (_count_diff_l / GEAR_RATIO / ENC_RES * ENC_L_DIR) / _delta_t * 60.0f;


        _v_r = (ENC_R_DIR * PI * _dia_tire / GEAR_RATIO / ENC_RES) *
                (float)_count_diff_r / _delta_t;

        _v_l = (ENC_L_DIR * PI * _dia_tire / GEAR_RATIO / ENC_RES) *
                (float)_count_diff_l / _delta_t;

        _v = (_v_r + _v_l) * 0.5f; 

        _yawrate_rad = (_v_r - _v_l) / _tread;
        _yawrate_deg = _yawrate_rad * 180.0f / PI;

        _v_r_list.push_front(_v_r);
        _v_r_list.pop_back();
        _v_l_list.push_front(_v_l);
        _v_l_list.pop_back();
        _v_r_ave = std::accumulate(_v_r_list.begin(), _v_r_list.end(), 0.0f) / (float)AVERAGE_NUM;
        _v_l_ave = std::accumulate(_v_l_list.begin(), _v_l_list.end(), 0.0f) / (float)AVERAGE_NUM;
        _rpm_r_ave = _v_r_ave * 60.0f / (PI * _dia_tire);
        _rpm_l_ave = _v_l_ave * 60.0f / (PI * _dia_tire);

        _publish();
    }

    void WheelOdometry::debug(){
        PRINTF_ASYNC("  count_r     : %d\n", _count_r);
        PRINTF_ASYNC("  count_l     : %d\n", _count_l);
        PRINTF_ASYNC("  count_r_pre : %d\n", _count_r_pre);
        PRINTF_ASYNC("  count_l_pre : %d\n", _count_l_pre);
        PRINTF_ASYNC("  ang_r_deg   : %f\n", _ang_r_deg);
        PRINTF_ASYNC("  ang_l_deg   : %f\n", _ang_l_deg);
        PRINTF_ASYNC("  rpm_r       : %f\n", _rpm_r);
        PRINTF_ASYNC("  rpm_l       : %f\n", _rpm_l);
        PRINTF_ASYNC("  rpm_r_ave   : %f\n", _rpm_r_ave);
        PRINTF_ASYNC("  rpm_l_ave   : %f\n", _rpm_l_ave);
        PRINTF_ASYNC("  ang_r_cor   : %f\n", _ang_r_cor);
        PRINTF_ASYNC("  ang_l_cor   : %f\n", _ang_l_cor);
        PRINTF_ASYNC("  v_r         : %f\n", _v_r);
        PRINTF_ASYNC("  v_l         : %f\n", _v_l);
        PRINTF_ASYNC("  v_r_ave     : %f\n", _v_r_ave);
        PRINTF_ASYNC("  v_l_ave     : %f\n", _v_l_ave);
        PRINTF_ASYNC("  v           : %f\n", _v);
        PRINTF_ASYNC("  yawrate_rad : %f\n", _yawrate_rad);
        PRINTF_ASYNC("  yawrate_deg : %f\n", _yawrate_deg);
    }

    void WheelOdometry::evalAng(float duty){
        constexpr uint32_t num = 400;
        float ang_r_list[num];
        float ang_l_list[num];

        Navigator::getInstance().setNavMode(ENavMode::DEBUG);
        hal::waitmsec(10);
        PRINTF_ASYNC("---------------------\n");
        PRINTF_ASYNC("time[sec], ang_l[deg], ang_r[deg]\n");

        PowerTransmission& pt = PowerTransmission::getInstance();
        pt.setDutyL(duty);
        pt.setDutyR(duty);
        hal::waitmsec(1000);
        for (uint16_t i = 0; i < num; i++) {            
            ang_r_list[i] = _ang_r_deg;
            ang_l_list[i] = _ang_l_deg;
            uint64_t start_usec = hal::getElapsedUsec();            
            while(1){
                uint64_t end_usec = hal::getElapsedUsec();
                if(end_usec - start_usec > 10000) break;
            }
        }
        pt.setDutyL(0.0f);
        pt.setDutyR(0.0f);

        for(uint16_t i = 0; i < num; i++){
            PRINTF_ASYNC("%f, %f, %f\n", (float)i*0.01f, ang_l_list[i], ang_r_list[i]);
            hal::waitmsec(10);
        }
    }

    void WheelOdometry::evalVelocity(float duty){
        constexpr uint32_t num = 400;
        float v_r_list[num];
        float v_l_list[num];

        Navigator::getInstance().setNavMode(ENavMode::DEBUG);
        hal::waitmsec(10);
        PRINTF_ASYNC("---------------------\n");
        PRINTF_ASYNC("time[sec], v_l[m/s], v_r[m/s]\n");

        PowerTransmission& pt = PowerTransmission::getInstance();
        pt.setDutyL(duty);
        pt.setDutyR(duty);
        hal::waitmsec(1000);
        for (uint16_t i = 0; i < num; i++) {            
            v_r_list[i] = _v_r;
            v_l_list[i] = _v_l;
            uint64_t start_usec = hal::getElapsedUsec();            
            while(1){
                uint64_t end_usec = hal::getElapsedUsec();
                if(end_usec - start_usec > 10000) break;
            }
        }
        pt.setDutyL(0.0f);
        pt.setDutyR(0.0f);

        for(uint16_t i = 0; i < num; i++){
            PRINTF_ASYNC("%f, %f, %f\n", (float)i*0.01f, v_l_list[i], v_r_list[i]);
            hal::waitmsec(10);
        }
    }

    void WheelOdometry::_publish(){
    	WheelOdometryMsg msg;
        msg.v = _v;
        msg.v_r = _v_r;
        msg.v_l = _v_l;
        msg.v_r_ave = _v_r_ave;
        msg.v_l_ave = _v_l_ave;

        msg.rpm_r = _rpm_r;
        msg.rpm_l = _rpm_l;
        msg.rpm_r_ave = _rpm_r_ave;
        msg.rpm_l_ave = _rpm_l_ave;

        msg.yawrate = _yawrate_rad;
        msg.ang_r = _ang_r_deg * PI / 180.0f;
        msg.ang_l = _ang_l_deg * PI / 180.0f;

        publishMsg(msg_id::WHEEL_ODOMETRY, &msg);
    }



    int usrcmd_wheelOdometry(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            WheelOdometry::getInstance().debug();
            return 0;
        }


        if (ntlibc_strcmp(argv[1], "eval") == 0){
            if(argc != 4){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            if(ntlibc_strcmp(argv[2], "ang") == 0){
                std::string duty_val_str(argv[3]);
                float duty_val = std::stof(duty_val_str);
                WheelOdometry::getInstance().evalAng(duty_val);
                return 0;
            }

            if(ntlibc_strcmp(argv[2], "v") == 0){
                std::string duty_val_str(argv[3]);
                float duty_val = std::stof(duty_val_str);
                WheelOdometry::getInstance().evalVelocity(duty_val);
                return 0;
            }

        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    };

}
