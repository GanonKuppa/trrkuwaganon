#include "controlMixer.h"

#include <cmath>

// Lib
#include "debugLog.h"
#include "ntlibc.h"
#include "pidController.h"

// Msg
#include "msgBroker.h"
#include "actuatorOutputMsg.h"
#include "pidControlValMsg.h"

// Module
#include "parameterManager.h"

namespace module{

    ControlMixer::ControlMixer() :
    _isLWall(false),
    _isRWall(false),
    _center_dist_r(0.0f),
    _center_dist_l(0.0f),
    _ahead_dist_r(0.0f),
    _ahead_dist_l(0.0f),
    _setp_v_xy_body(0.0f),
    _setp_a_xy_body(0.0f),
    _setp_yaw(0.0f),
    _setp_yawrate(0.0f),
    _setp_yawacc(0.0f),
    _traj_type_pre(ETrajType::STOP),
    _traj_type(ETrajType::STOP),
    _turn_type_pre(ETurnType::STOP),
    _turn_type(ETurnType::STOP),
    _duty_r(0.0f),
    _duty_l(0.0f),
    _duty_r_v(0.0f),
    _duty_l_v(0.0f),
    _duty_r_yaw(0.0f),
    _duty_l_yaw(0.0f),
    _error_sec(0.0f),
    _is_error(false)
    {
        setModuleName("ControlMixer");
        _wall_pidf.reset();
        _wall_diag_pidf.reset();
        _yaw_pidf.reset();
        _yawrate_pidf.reset();
        _v_pidf.reset();
    }

    void ControlMixer::update0(){
        _msgUpdate();
        _updateControllerParam();
        _updateController();
        if(_nav_msg.mode == ENavMode::STANDBY || _nav_msg.mode == ENavMode::FASTEST || _nav_msg.mode == ENavMode::SEARCH){
            _publish();
        }
    }

    void ControlMixer::debug(){
        PRINTF_ASYNC("  duty_r     : %f\n", _duty_r);
        PRINTF_ASYNC("  duty_l     : %f\n", _duty_l);
        PRINTF_ASYNC("  duty_r_v   : %f\n", _duty_r_v);
        PRINTF_ASYNC("  duty_l_v   : %f\n", _duty_l_v);
        PRINTF_ASYNC("  duty_r_yaw : %f\n", _duty_r_yaw);
        PRINTF_ASYNC("  duty_l_yaw : %f\n", _duty_l_yaw);
        PRINTF_ASYNC("  traj_type  : %s\n", trajType2Str(_traj_type).c_str());
        PRINTF_ASYNC("  turn_type  : %s\n", turnType2Str(_turn_type).c_str());
    }


    void ControlMixer::_msgUpdate(){
        copyMsg(msg_id::CTRL_SETPOINT, &_setp_msg);
        copyMsg(msg_id::NAV_STATE, &_nav_msg);
        copyMsg(msg_id::WALL_SENSOR, &_ws_msg);
        copyMsg(msg_id::VEHICLE_ATTITUDE, &_att_msg);
        copyMsg(msg_id::VEHICLE_POSITION, &_pos_msg);

        _traj_type_pre = _traj_type;
        _turn_type_pre = _turn_type;
        _traj_type = _setp_msg.traj_type;
        _turn_type = _setp_msg.turn_type;
    }

    void ControlMixer::_updateControllerParam(){
            ParameterManager& pm = ParameterManager::getInstance();

            // 速度, 角速度, 角度PIDFゲイン設定
            if(_traj_type == ETrajType::STOP || _traj_type == ETrajType::SPINTURN) {
                _v_pidf.set(pm.spin_v_p, pm.spin_v_i, pm.spin_v_d, pm.spin_v_f);
                _yawrate_pidf.set(pm.spin_yawrate_p, pm.spin_yawrate_i, pm.spin_yawrate_d, pm.spin_yawrate_f);
                _yaw_pidf.set(pm.spin_yaw_p, pm.spin_yaw_i, 0.0f, 0.0f);

            } else if(_nav_msg.mode == ENavMode::FASTEST) {
                _v_pidf.set(pm.fast_v_p, pm.fast_v_i, pm.fast_v_d, pm.fast_v_f);
                _yawrate_pidf.set(pm.fast_yawrate_p, pm.fast_yawrate_i, pm.fast_yawrate_d, pm.fast_yawrate_f);
                _yaw_pidf.set(pm.fast_yaw_p, pm.fast_yaw_i, 0.0f, 0.0f);
            } else if(_nav_msg.mode == ENavMode::SEARCH) {
                _v_pidf.set(pm.search_v_p, pm.search_v_i, pm.search_v_d, pm.search_v_f);
                _yawrate_pidf.set(pm.search_yawrate_p, pm.search_yawrate_i, pm.search_yawrate_d, pm.search_yawrate_f);
                _yaw_pidf.set(pm.search_yaw_p, pm.search_yaw_i, 0.0f, 0.0f);
            } else {
                _v_pidf.set(pm.search_v_p, pm.search_v_i, pm.search_v_d, pm.search_v_f);
                _yawrate_pidf.set(pm.search_yawrate_p, pm.search_yawrate_i, pm.search_yawrate_d, pm.search_yawrate_f);
                _yaw_pidf.set(pm.search_yaw_p, pm.search_yaw_i, 0.0f, 0.0f);
            }
            
            // 壁PIDFゲイン設定
            _wall_pidf.set(pm.wall_p, pm.wall_i, pm.wall_d, pm.wall_f);

            // 斜めPIDFゲイン設定
            _wall_pidf.set(pm.wall_diag_p, pm.wall_diag_i, pm.wall_diag_d, pm.wall_diag_f);
            
            // FB制御イネーブル設定
            _v_pidf.setEnable(pm.v_fb_enable);
            _yawrate_pidf.setEnable(pm.yawrate_fb_enable);
            _yaw_pidf.setEnable(pm.yaw_fb_enable);
            _wall_pidf.setEnable(pm.wall_fb_enable);
            _wall_diag_pidf.setEnable(pm.wall_diag_fb_enable);

            // サチュレーションenable設定
            if(_traj_type == ETrajType::STOP || _traj_type == ETrajType::SPINTURN) {
                _v_pidf.setSaturationEnable(false);
                _yawrate_pidf.setSaturationEnable(false);                
            }
            else{
                _v_pidf.setSaturationEnable(pm.v_saturation_enable);
                _yawrate_pidf.setSaturationEnable(pm.yawrate_saturation_enable);
            }
            
            _yaw_pidf.setSaturationEnable(pm.yaw_saturation_enable);
            _wall_pidf.setSaturationEnable(pm.wall_saturation_enable);
            _wall_diag_pidf.setSaturationEnable(pm.wall_diag_saturation_enable);

            // サチュレーション値設定
                //_v_pidfおよび_yawrate_pidfのサチュレーション値はupdateController()で設定
            _yaw_pidf.setSaturation(pm.yaw_saturation_enable);
            _wall_pidf.setSaturation(pm.wall_saturation_enable);
            _wall_diag_pidf.setSaturation(pm.wall_diag_saturation);
                        
            // 積分サチュレーションイネーブル設定
            _v_pidf.setIntegralSaturationEnable(pm.v_i_saturation_enable);
            _yawrate_pidf.setIntegralSaturationEnable(pm.yawrate_i_saturation_enable);
            _yaw_pidf.setIntegralSaturationEnable(pm.yaw_i_saturation_enable);
            _wall_pidf.setIntegralSaturationEnable(pm.wall_i_saturation_enable);
            _wall_diag_pidf.setIntegralSaturationEnable(pm.wall_diag_i_saturation_enable);
            
            // 積分サチュレーション値設定
            _v_pidf.setIntegralSaturation(pm.v_i_saturation);
            _yawrate_pidf.setIntegralSaturation(pm.yawrate_i_saturation);
            _yaw_pidf.setIntegralSaturation(pm.yaw_i_saturation);
            _wall_pidf.setIntegralSaturation(pm.wall_i_saturation);
            _wall_diag_pidf.setIntegralSaturation(pm.wall_diag_i_saturation);
    }

    void ControlMixer::_updateController(){
        if(_nav_msg.mode != ENavMode::STANDBY && _nav_msg.mode != ENavMode::FASTEST && _nav_msg.mode != ENavMode::SEARCH){
            _v_pidf.reset();
            _yawrate_pidf.reset();
            _yaw_pidf.reset();
            _wall_pidf.reset();
            _wall_diag_pidf.reset();

            return;
        }        
        
        ParameterManager& pm = ParameterManager::getInstance();
        
        _setp_v_xy_body = _setp_msg.v_xy_body;
        _setp_yaw = _setp_msg.yaw;
        _setp_yawrate = _setp_msg.yawrate;

        // 超信知 or 停止制御後は制御量をリセットする
        if( _traj_type != _traj_type_pre &&
            (_traj_type_pre == ETrajType::STOP || _traj_type_pre == ETrajType::SPINTURN)
        ) {
            _v_pidf.reset();
            _yawrate_pidf.reset();
            _yaw_pidf.reset();
            _wall_pidf.reset();
            _wall_diag_pidf.reset();
        }

        // 壁制御
        if(_turn_type == ETurnType::STRAIGHT_CENTER || _turn_type == ETurnType::STRAIGHT_CENTER_EDGE) {
            
            if(_nav_msg.mode == ENavMode::SEARCH || _nav_msg.mode == ENavMode::FASTEST){
                _wall_pidf.update(_ws_msg.dist_r, _ws_msg.dist_l, _nav_msg.r_wall_enable, _nav_msg.l_wall_enable);
            }
            else if(_nav_msg.mode == ENavMode::STANDBY){
                _wall_pidf.update(_ws_msg.dist_r, _ws_msg.dist_l, true, true);
            }
            
            // 壁制御量は曲率とみなし, 速度をかけることで角速度に変換
            float v_now = _pos_msg.v_xy_body_for_ctrl;
            if(v_now < 0.1f) v_now = 0.1f;            
            _setp_yawrate += v_now * _wall_pidf.getControlVal();
        }
        else{
            _wall_pidf.reset();
        }

        // 斜め直進時の壁制御
        if(_turn_type == ETurnType::DIAGONAL_CENTER) {
            float v_now = _pos_msg.v_xy_body_for_ctrl;
            if(v_now < 0.1f) v_now = 0.1f;              

            if (_ws_msg.ahead_l < pm.diag_corner_threshold_off_l &&
                _ws_msg.ahead_r >= pm.diag_corner_threshold_on_r
            ) {
                //setp_yawrate -= v_now * pm.wall_diag_p;
                _wall_diag_pidf.update(_ws_msg.ahead_r, pm.diag_corner_threshold_off_r);
                _setp_yawrate += - v_now * _wall_diag_pidf.getControlVal();
            } 
            else if (_ws_msg.ahead_l >= pm.diag_corner_threshold_on_l &&
            		 _ws_msg.ahead_r < pm.diag_corner_threshold_off_r
            ) {
                //setp_yawrate += v_now * pm.wall_diag_p;
                _wall_diag_pidf.update(_ws_msg.ahead_l, pm.diag_corner_threshold_off_l);
                _setp_yawrate += v_now * _wall_diag_pidf.getControlVal();
            }
            else{
                _wall_diag_pidf.reset();
            }            
        }
        else{
            _wall_diag_pidf.reset();
        }

        // 角度制御
        if(_wall_pidf.engaged()){
            _yaw_pidf.reset();
        }
        else{
            _yaw_pidf.update(_setp_yaw, _att_msg.yaw);
            _setp_yawrate += _yaw_pidf.getControlVal();
        }

        // 速度, 角速度制御量更新
        _v_pidf.update(_setp_v_xy_body, _pos_msg.v_xy_body_for_ctrl);
        _yawrate_pidf.update(_setp_yawrate, _att_msg.yawrate);

        _duty_r = 0.0f;
        _duty_l = 0.0f;
        _duty_r_v = 0.0f;
        _duty_l_v = 0.0f;
        _duty_r_yaw = 0.0f;
        _duty_l_yaw = 0.0f;
        float duty_v_saturation = 0.0f;
        float duty_yawrate_saturation = 0.0f;
        
        // 速度FF制御
        float v_ff = pm.v_ff_coef * _setp_v_xy_body + pm.v_ff_offset;
        float a_ff = pm.a_ff_coef * _setp_a_xy_body + pm.a_ff_offset;

        if(pm.v_ff_enable && _setp_v_xy_body > 0.01f){    
            _duty_r_v += v_ff + a_ff;
            _duty_l_v += v_ff + a_ff;
            duty_v_saturation = std::min(pm.v_saturation_ff_multiplier * (v_ff + a_ff), pm.v_saturation_offset_duty);
        }else{
            duty_v_saturation = pm.v_saturation_offset_duty;
        }

        // 角速度FF制御
        float yawrate_ff = pm.yawrate_ff_coef * std::fabs(_setp_yawrate) + pm.yawrate_ff_offset;
        float yawacc_ff = pm.yawacc_ff_coef * std::fabs(_setp_yawacc) + pm.yawacc_ff_offset;

        if(pm.yawrate_ff_enable && std::fabs(_setp_yawrate) > 0.52f){ // 0.52 rad/s = 30 deg/s
            float rate_sign = _sign(_setp_yawrate);
            float acc_sign = _sign(_setp_yawacc);
            
            _duty_r_yaw +=    rate_sign * yawrate_ff + acc_sign * yawacc_ff;
            _duty_l_yaw += - (rate_sign * yawrate_ff + acc_sign * yawacc_ff);
            float ff_saturation = pm.yawrate_saturation_ff_multiplier * std::fabs(rate_sign * yawrate_ff + acc_sign * yawacc_ff);
            duty_yawrate_saturation = std::min(ff_saturation, pm.yawrate_saturation_offset_duty);
        }else{
            duty_yawrate_saturation = pm.yawrate_saturation_offset_duty;
        }

        // 速度、角速度saturation
        if(_traj_type == ETrajType::STOP || _traj_type == ETrajType::SPINTURN) {
            // do nothing
        }else{
            _v_pidf.setSaturation(duty_v_saturation);
            _yawrate_pidf.setSaturation(duty_yawrate_saturation);
        }

        // dutyのセット
        _duty_r_v += _v_pidf.getControlVal();
        _duty_l_v += _v_pidf.getControlVal();
        _duty_r_yaw +=   _yawrate_pidf.getControlVal();
        _duty_l_yaw += - _yawrate_pidf.getControlVal();
        _duty_r = _duty_r_v + _duty_r_yaw;
        _duty_l = _duty_l_v + _duty_l_yaw;

        // 制御誤差の監視
        if( std::fabs(_yawrate_pidf.getError()) > YAWRATE_ERROR_TH || std::fabs(_v_pidf.getError()) > std::fabs(V_ERROR_TH)){
            _error_sec += _delta_t;
        }                
        else{
            _error_sec = 0.0f;
        }            
        _is_error = (_error_sec > ERROR_TIME_TH);
    }

    void ControlMixer::_publish(){
        ActuatorOutputMsg out_msg;
        PidControlValMsg pid_msg;
        out_msg.duty_r = _duty_r;
        out_msg.duty_l = _duty_l;
        out_msg.duty_r_v = _duty_r_v;
        out_msg.duty_l_v = _duty_l_v;
        out_msg.duty_r_yaw = _duty_r_yaw;
        out_msg.duty_l_yaw = _duty_l_yaw;
        out_msg.is_error = _is_error;

        out_msg.ctrl_mode = ECtrlMode::VEHICLE;

        pid_msg.v_p = _v_pidf.getPVal();
        pid_msg.v_i = _v_pidf.getIVal();
        pid_msg.v_d = _v_pidf.getDVal();
        pid_msg.yawrate_p = _yawrate_pidf.getPVal();
        pid_msg.yawrate_i = _yawrate_pidf.getIVal();
        pid_msg.yawrate_d = _yawrate_pidf.getDVal();
        pid_msg.yaw_p = _yaw_pidf.getPVal();
        pid_msg.yaw_i = _yaw_pidf.getIVal();
        pid_msg.yaw_d = _yaw_pidf.getDVal();
        pid_msg.wall_p = _wall_pidf.getPVal();
        pid_msg.wall_i = _wall_pidf.getIVal();
        pid_msg.wall_d = _wall_pidf.getDVal();
        pid_msg.diag_p = _wall_diag_pidf.getPVal();
        pid_msg.diag_i = _wall_diag_pidf.getIVal();
        pid_msg.diag_d = _wall_diag_pidf.getDVal();

        pid_msg.setp_v_xy_body = _setp_v_xy_body;
        pid_msg.setp_yaw = _setp_yaw;
        pid_msg.setp_yawrate = _setp_yawrate;

        publishMsg(msg_id::ACTUATOR_OUTPUT, &out_msg);
        publishMsg(msg_id::PID_CONTROL_VAL, &pid_msg);
    }

    float ControlMixer::_sign(float val){
        if(val > 0.0f) return 1.0f;
        else if(val < 0.0f) return -1.0f;
        else return 0.0f;
    }

    int usrcmd_controlMixer(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            ControlMixer::getInstance().debug();
            return 0;
        }        
        
        return 0;
    };
}
