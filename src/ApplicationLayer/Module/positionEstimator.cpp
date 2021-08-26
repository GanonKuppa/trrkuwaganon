#include "positionEstimator.h"
#include "vehicleAttitudeMsg.h"
#include "vehiclePositionMsg.h"
#include "msgBroker.h"


namespace module {

    PositionEstimator::PositionEstimator() :
	_x(0.0f),
	_y(0.0f),
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
	_beta(0.0f),
	_yawrate(0.0f),
	_rollrate(0.0f),
	_pitchrate(0.0f),
	_after_curve_time(0.0f)
    {        
        setModuleName("PositionEstimator");

        for (uint8_t i = 0; i < ACC_Y_AVERAGE_NUM; i++) {
            _acc_y_list.push_front(0.0f);
        }    
    }

    void PositionEstimator::reset(){
    }

    void PositionEstimator::update0(){

    }

    void PositionEstimator::_publish_vehicle_position(){
        VehiclePositionMsg msg;
        msg.x = _x;
        msg.y = _y;
        msg.z = _z;

        msg.v_xy_body_cmp = _v_xy_body_cmp;
        msg.v_xy_body_enc = _v_xy_body_enc;
        msg.v_xy_body_enc = _v_xy_body_ave;
        msg.v_xy_body_acc = _v_xy_body_acc;

        msg.x = _v_x;
        msg.y = _v_y;
        msg.z = _v_z;

        msg.v_xy_body_for_odom = _v_xy_body_for_odom;
        msg.v_xy_body_for_odom = _v_xy_body_for_ctrl;

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

        msg.yawrate = _yawrate;
        msg.rollrate = _rollrate;
        msg.pitchrate = _pitchrate;

        publishMsg(msg_id::VEHICLE_ATTITUDE, &msg);
    }


    int usrcmd_positionEstimator(int argc, char **argv){
            return 0;
    }

}
