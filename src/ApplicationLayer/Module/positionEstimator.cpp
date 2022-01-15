#include "positionEstimator.h"

#include <cmath>
#include <deque>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Msg
#include "msgBroker.h"
#include "ctrlSetpointMsg.h"
#include "imuMsg.h"
#include "wheelOdometryMsg.h"
#include "vehicleAttitudeMsg.h"
#include "vehiclePositionMsg.h"
#include "wallSensorMsg.h"
#include "navStateMsg.h"

// Obj
#include "navigationEnum.h"


namespace module {

    PositionEstimator::PositionEstimator() :
	_x(0.045f),
	_y(0.045f),
	_z(0.0f),
	_v_xy_body_cmp(0.0f),
	_v_xy_body_enc(0.0f),
	_v_xy_body_ave(0.0f),
	_v_xy_body_acc(0.0f),
	_v_x(0.0f),
	_v_y(0.0f),
	_v_z(0.0f),
	_v_xy_body_for_odom(0.0f),
	_v_xy_body_for_ctrl(0.0f),
	_a_body_x(0.0f),
	_a_body_y(0.0f),
	_a_body_z(0.0f),
	_yaw(90.0f * DEG2RAD),
	_roll(0.0f),
	_pitch(0.0f),
	_q(Eigen::Quaternionf::Identity()),
    _roll_acc(0.0f),
    _pitch_acc(0.0f),
    _beta(0.0f),
    _beta_dot(0.0f),
	_yawrate(0.0f),
	_rollrate(0.0f),
	_pitchrate(0.0f),
    _turn_type(ETurnType::NONE),
	_beta_expiration_time(0.0f),
    _on_wall_center_dist(0.0f),
    _in_read_wall_area_pre(false)
    {        
        setModuleName("PositionEstimator");
        
        for (uint8_t i = 0; i < ACC_Y_AVERAGE_NUM; i++) {
            _acc_y_list.push_front(0.0f);
            _v_enc_list.push_front(0.0f);
        }    
    }

    void PositionEstimator::reset(float x, float y, float yaw){
        _x = x;
        _y = y;
        _z = 0.0f;
        _yaw = yaw;
        _beta = 0.0f;
        _beta_dot = 0.0f;
        _beta_expiration_time = 0.0f;

        for (uint8_t i = 0; i < ACC_Y_AVERAGE_NUM; i++) {
            _acc_y_list[i] = 0.0f;
            _v_enc_list[i] = 0.0f;
        }    
    }

    void PositionEstimator::update0(){
        ImuMsg imu_msg;
        WheelOdometryMsg wodo_msg;
        CtrlSetpointMsg ctrl_msg;
        WallSensorMsg ws_msg;
        NavStateMsg nav_msg;
        copyMsg(msg_id::IMU, &imu_msg);
        copyMsg(msg_id::WHEEL_ODOMETRY, &wodo_msg);
        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        copyMsg(msg_id::WALL_SENSOR, &ws_msg);
        copyMsg(msg_id::NAV_STATE, &nav_msg);

        float yawrate_pre = _yawrate; // yawrateは今期に取得したものは今期のデータ
        float yaw_pre = _yaw;
        float v_x_pre = _v_x;
        float v_y_pre = _v_y;
        float v_xy_body_for_odom_pre = _v_xy_body_for_odom; //エンコーダ計測速度は今期に取得できるものが前期のデータ
        float a_body_x_pre = _a_body_x;
        float a_body_y_pre = _a_body_y;
        float a_body_z_pre = _a_body_z;
        float beta_pre = _beta;
        ETurnType turn_type_pre = _turn_type;

        _turn_type = ctrl_msg.turn_type;

        _a_body_x = imu_msg.acc_x;
        _a_body_y = imu_msg.acc_y;
        _a_body_z = imu_msg.acc_z;

        _acc_y_list.push_front(imu_msg.acc_y);
        _acc_y_list.pop_back();
        _v_enc_list.push_front(wodo_msg.v);
        _v_enc_list.pop_back();

        // 角度算出
        _yawrate = imu_msg.yawrate;
        _yaw += yawrate_pre * _delta_t;
        _yaw = fmodf(_yaw + 2.0 * PI, 2.0 * PI);
        
        // 並進速度算出
        float acc_y_sum = 0.0f;
        float v_sum = 0.0f;
        for (uint8_t i=0; i< ACC_Y_AVERAGE_NUM; i++){
            acc_y_sum += _acc_y_list[i];
            v_sum += _v_enc_list[i];
        }
        
        _v_xy_body_enc = wodo_msg.v;         
        _v_xy_body_ave = v_sum / (float)ACC_Y_AVERAGE_NUM;
        _v_xy_body_cmp = _v_xy_body_ave + acc_y_sum * (_delta_t * 0.5f);
        _v_xy_body_for_odom = _v_xy_body_enc;
        _v_xy_body_for_ctrl = _v_xy_body_enc;
        
        // 加速度積分速度算出
        _v_xy_body_acc += imu_msg.acc_y * _delta_t;
        if(std::fabs(_v_xy_body_enc) < 0.01f){
            _v_xy_body_acc = _v_xy_body_enc;
        }

        // グローバル座標系速度算出
        _v_x = _v_xy_body_for_odom * cosf(yaw_pre); // 一旦betaを0に
        _v_y = _v_xy_body_for_odom * sinf(yaw_pre); // 

        // グローバル座標系位置算出
        _x += _delta_t * v_x_pre;
        _y += _delta_t * v_y_pre;

        // スリップ角算出
        _beta_expiration_time -= _delta_t;
        if(_beta_expiration_time < 0.0f){
            _beta_expiration_time = 0.0f;
        }
        
        if(ctrl_msg.traj_type == ETrajType::CURVE){
            _beta_expiration_time = _afrer_curve_beta_expiration_time;
        }
        
        if(ctrl_msg.traj_type == ETrajType::CURVE || _beta_expiration_time > 0.0f){
            _beta_dot = -a_body_x_pre / _v_xy_body_for_odom - yawrate_pre;
            _beta += _beta_dot * _delta_t;            
        }
        else{
            _beta_dot = 0.0f;
            _beta = 0.0f;
        }


        // 壁中心判定
        if((_turn_type == ETurnType::STRAIGHT_CENTER || _turn_type == ETurnType::STRAIGHT_CENTER) && 
            ws_msg.is_on_wall_center &&
            !ws_msg.is_ahead && 
            _v_xy_body_for_odom > 0.1f && 
            std::fabs(_yawrate) < 50.0f * DEG2RAD
        ){
            _on_wall_center_dist += _v_xy_body_for_odom * _delta_t;
        }
        else{
            _on_wall_center_dist = 0.0f;
        }

        // 前壁補正時に位置, 方位を強制書き換え
        if(turn_type_pre == ETurnType::AHEAD_WALL_CORRECTION && _turn_type != ETurnType::AHEAD_WALL_CORRECTION){
            _aheadWallCorrection();
        }

        // 迷路の壁読み時に前壁からの距離で位置を補正
        bool in_read_wall_area = nav_msg.in_read_wall_area;
        if(!in_read_wall_area && _in_read_wall_area_pre && ws_msg.is_ahead && nav_msg.mode == ENavMode::SEARCH){
            _aheadWallCorrectionOnWallRead(ws_msg.dist_a);
        }
        _in_read_wall_area_pre = in_read_wall_area;

        _onWallCenterCorrection();

        _publish_vehicle_position();
        _publish_vehicle_attitude();
    }

    void PositionEstimator::_onWallCenterCorrection() {
        float x_pre = _x;
        float y_pre = _y;
        float yaw_pre = _yaw;

        float ang = _yaw * RAD2DEG;
        if(_on_wall_center_dist > 0.06f) {            
            if(ang >= 315.0 || ang < 45.0) {
                _y = (uint8_t)(_y / 0.09f) * 0.09f + 0.09f/2.0f;
                _yaw = 0.0 * DEG2RAD;
            } else if(ang >= 45.0 && ang < 135.0) {
                _x = (uint8_t)(_x / 0.09f) * 0.09f + 0.09f/2.0f;
                _yaw = 90.0f * DEG2RAD;
            } else if(ang >= 135.0f && ang < 225.0f) {
                _y = (uint8_t)(_y / 0.09f) * 0.09f + 0.09f/2.0f;
                _yaw = 180.0 * DEG2RAD;
            } else if(ang >= 225.0f && ang < 315.0f) {
                _x = (uint8_t)(_x / 0.09f) * 0.09f + 0.09f/2.0f;
                _yaw = 270.0f * DEG2RAD;
            }
            _on_wall_center_dist = 0.0f;
            PRINTF_PICKLE("ON_WALL_CENTER | x:%6.3f, x_pre:%6.3f, y:%6.3f, y_pre:%6.3f yaw:%6.3f, yaw_pre:%6.3f\n", _x, x_pre, _y, y_pre, _yaw*RAD2DEG, yaw_pre*RAD2DEG);
        }
    }

    void PositionEstimator::_aheadWallCorrection() {
        float x_pre = _x;
        float y_pre = _y;
        float yaw_pre = _yaw;

        float ang = _yaw * RAD2DEG;
        if(ang >= 315.0 || ang < 45.0) {
            _x = (uint8_t)(_x / 0.09f) * 0.09f + 0.09f/2.0f;
            _yaw = 0.0 * DEG2RAD;
        } else if(ang >= 45.0 && ang < 135.0) {
            _y = (uint8_t)(_y / 0.09f) * 0.09f + 0.09f/2.0f;
            _yaw = 90.0f * DEG2RAD;
        } else if(ang >= 135.0f && ang < 225.0f) {
            _x = (uint8_t)(_x / 0.09f) * 0.09f + 0.09f/2.0f;
            _yaw = 180.0 * DEG2RAD;
        } else if(ang >= 225.0f && ang < 315.0f) {
            _y = (uint8_t)(_y / 0.09f) * 0.09f + 0.09f/2.0f;
            _yaw = 270.0f * DEG2RAD;
        }
        _on_wall_center_dist = 0.0f;
        PRINTF_PICKLE("AHEAD_WALL_CORRECTION | x:%6.3f, x_pre:%6.3f, y:%6.3f, y_pre:%6.3f yaw:%6.3f, yaw_pre:%6.3f\n", _x, x_pre, _y, y_pre, _yaw*RAD2DEG, yaw_pre*RAD2DEG);
    }

    void PositionEstimator::_aheadWallCorrectionOnWallRead(float dist_a) {
        float ang = _yaw * RAD2DEG;
        if(ang >= 315.0 || ang < 45.0) {
            _x = (uint8_t)(_x / 0.09f) * 0.09f + 0.09f - dist_a;            
        } else if(ang >= 45.0 && ang < 135.0) {
            _y = (uint8_t)(_y / 0.09f) * 0.09f + 0.09f - dist_a;            
        } else if(ang >= 135.0f && ang < 225.0f) {
            _x = (uint8_t)(_x / 0.09f) * 0.09f + dist_a;
        } else if(ang >= 225.0f && ang < 315.0f) {
            _y = (uint8_t)(_y / 0.09f) * 0.09f + dist_a;            
        }
    }



    void PositionEstimator::debug(){
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  x                    : %f\n", _x);
        PRINTF_ASYNC(  "  y                    : %f\n", _y);
        PRINTF_ASYNC(  "  z                    : %f\n", _z);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  v_xy_body_cmp        : %f\n", _v_xy_body_cmp);
        PRINTF_ASYNC(  "  v_xy_body_enc        : %f\n", _v_xy_body_enc);
        PRINTF_ASYNC(  "  v_xy_body_ave        : %f\n", _v_xy_body_ave);
        PRINTF_ASYNC(  "  v_xy_body_acc        : %f\n", _v_xy_body_acc);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  v_x                  : %f\n", _v_x);
        PRINTF_ASYNC(  "  v_y                  : %f\n", _v_y);
        PRINTF_ASYNC(  "  v_z                  : %f\n", _v_z);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  v_xy_body_for_odom   : %f\n", _v_xy_body_for_odom);
        PRINTF_ASYNC(  "  v_xy_body_for_ctrl   : %f\n", _v_xy_body_for_ctrl);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  yaw                  : %f\n", _yaw * RAD2DEG);
        PRINTF_ASYNC(  "  roll                 : %f\n", _roll * RAD2DEG);
        PRINTF_ASYNC(  "  pitch                : %f\n", _pitch * RAD2DEG);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  yawrate              : %f\n", _yawrate * RAD2DEG);
        PRINTF_ASYNC(  "  rollrate             : %f\n", _rollrate * RAD2DEG);
        PRINTF_ASYNC(  "  pitchrate            : %f\n", _pitchrate * RAD2DEG);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  beta                 : %f\n", _beta * RAD2DEG);
        PRINTF_ASYNC(  "  beta_dot             : %f\n", _beta_dot * RAD2DEG);
        PRINTF_ASYNC(  "  beta_expiration_time : %f\n", _beta_expiration_time);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  a_body_x             : %f\n", _a_body_x);
        PRINTF_ASYNC(  "  a_body_y             : %f\n", _a_body_y);
        PRINTF_ASYNC(  "  a_body_z             : %f\n", _a_body_z);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  roll_acc             : %f\n", _roll_acc * RAD2DEG);
        PRINTF_ASYNC(  "  pitch_acc            : %f\n", _pitch_acc * RAD2DEG);
        PRINTF_ASYNC(  "  ---\n");
        PRINTF_ASYNC(  "  q0                   : %f\n", _q.w());
        PRINTF_ASYNC(  "  q1                   : %f\n", _q.x());
        PRINTF_ASYNC(  "  q2                   : %f\n", _q.y());
        PRINTF_ASYNC(  "  q3                   : %f\n", _q.z());
    }

    void PositionEstimator::_publish_vehicle_position(){
        VehiclePositionMsg msg;
        msg.x = _x;
        msg.y = _y;
        msg.z = _z;

        msg.v_xy_body_cmp = _v_xy_body_cmp;
        msg.v_xy_body_enc = _v_xy_body_enc;
        msg.v_xy_body_ave = _v_xy_body_ave;
        msg.v_xy_body_acc = _v_xy_body_acc;

        msg.v_x = _v_x;
        msg.v_y = _v_y;
        msg.v_z = _v_z;

        msg.v_xy_body_for_odom = _v_xy_body_for_odom;
        msg.v_xy_body_for_ctrl = _v_xy_body_for_ctrl;

        msg.a_body_x = _a_body_x;
        msg.a_body_y = _a_body_y;
        msg.a_body_z = _a_body_z;

        publishMsg(msg_id::VEHICLE_POSITION, &msg);
    }


    void PositionEstimator::_publish_vehicle_attitude(){
        VehicleAttitudeMsg msg;

        msg.yaw = _yaw;
        msg.roll = _roll;
        msg.pitch = _pitch;

        msg.q0 = _q.w();
        msg.q1 = _q.x();
        msg.q2 = _q.y();
        msg.q3 = _q.z();

        msg.roll_acc = _roll_acc;
        msg.pitch_acc = _pitch_acc;

        msg.beta = _beta;
        msg.beta_dot = _beta_dot;

        msg.yawrate = _yawrate;
        msg.rollrate = _rollrate;
        msg.pitchrate = _pitchrate;

        publishMsg(msg_id::VEHICLE_ATTITUDE, &msg);
    }


    int usrcmd_positionEstimator(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status :\r\n");
            PRINTF_ASYNC("  reset  :\r\n");
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            PositionEstimator::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "reset") == 0) {
        	constexpr float DEG2RAD = 3.14159265f/180.0f;
            PositionEstimator::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);
            return 0;
        }


        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    }

}
