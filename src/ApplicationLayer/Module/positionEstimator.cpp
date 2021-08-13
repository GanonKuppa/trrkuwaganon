#include "positionEstimator.h"

namespace module {

    PositionEstimator::PositionEstimator() :
    _v_body(0.0f),
    _v_body_enc(0.0f),
    _v_body_acc(0.0f),
    _v_body_for_odom(0.0f),
    _yaw(90.0f * DEG2RAD),
    _roll(0.0f),
    _pitch(0.0f),
    _beta(0.0f),
    _yawrate(0.0f),
    _rollrate(0.0f),
    _pitchrate(0.0f),
    _x(0.045f),
    _y(0.045f),
    _z(0.0f),
    _v_x(0.0f),
    _v_y(0.0f),
    _v_z(0.0f),
    _a_body_x(0.0f),
    _a_body_y(0.0f),
    _a_body_z(0.0f),
    _a_x(0.0f),
    _a_y(0.0f),
    _a_z(0.0f),
    _after_curve_time(0.0f)
    {        
        for (uint8_t i = 0; i < ACC_Y_AVERAGE_NUM; i++) {
            _acc_y_list.push_front(0.0f);
        }    
    }

    PositionEstimator::reset(){
        _v_body = 0.0f;
        _v_body_enc = 0.0f;
        _v_body_acc = 0.0f;
        _v_body_for_odom = 0.0f;

        _yaw = 90.0f * DEG2RAD;
        _roll = 0.0f;
        _pitch = 0.0f;

        //_q;

        _beta = 0.0f;

        _yawrate = 0.0f;
        _rollrate = 0.0f;
        _pitchrate = 0.0f;

        _x = 0.045f;
        _y = 0.045f;
        _z = 0.0f;

        _v_x = 0.0f;
        _v_y = 0.0f;
        _v_z = 0.0f;

        _a_body_x = 0.0f;
        _a_body_y = 0.0f;
        _a_body_z = 0.0f;

        _a_x = 0.0f;
        _a_y = 0.0f;
        _a_z = 0.0f;

        _after_curve_time = 0.0f;
    }

    PositionEstimator::update0(){
        _
    }

    PositionEstimator::_publish(){
        PositionEstimatorMsg msg;
        msg.v_xy_body = _v_xy_body;
        msg.v_xy_body_enc = _v_xy_body_enc;
        msg.v_xy_body_acc = _v_xy_body_acc;
        msg.v_xy_body_for_odom = _v_xy_body_for_odom;

        msg.yaw = _yaw;
        msg.roll = _roll;
        msg.pitch = _pitch;

        msg.q0 = 1.0f;
        msg.q1 = 0.0f;
        msg.q2 = 0.0f;
        msg.q3 = 0.0f;

        msg.roll_acc = 0.0f;
        msg.pitch_acc = 0.0f;

        msg.beta = _beta;

        msg.yawrate = _yawrate;
        msg.rollrate = _rollrate;
        msg.pitchrate = _pitchrate;

        msg.x = _x;
        msg.y = _y;
        msg.z = _z;

        publishMsg(msg_id::POSITION_ESTIMATOR, &msg);

    }

    int usrcmd_positionEstimator(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            PositionEstimator::getInstance().debug();
            return 0;
        }    
    }

}