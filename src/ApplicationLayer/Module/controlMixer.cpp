#include "controlMixer.h"

namespace module{

    ControlMixer::ControlMixer(){
        setModuleName("ControlMixer");
    }

    ControlMixer::update(){
        ParameterManager& pm = ParameterManager::getInstance();
        EMotionType motion_type = traj.motion_type;
        turn_type_e turn_type = traj.turn_type;
        setPIDF(motion_type);
        target_trans_v = traj.v;
        target_trans_a = traj.a;

        target_rot_x = traj.ang;
        target_rot_v = traj.ang_v;
        target_rot_a = traj.ang_a;


        //壁制御
        if(motion_type == EMotionType::STRAIGHT_WALL_CENTER) {
            wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall, isPillar);
        } else {
            wall_pidf.reset();
        }

        if( (isRWall || isLWall) &&
                motion_type == EMotionType::STRAIGHT_WALL_CENTER &&
                pm.wall_PIDF_enable == true
            ) {
            wall_pidf.update(WallSensor::getInstance(), isRWall, isLWall, isPillar);
            // 壁制御量は曲率とみなし, 速度をかけることで角速度に変換
            float v_now = constrainL(esti.getV(), 0.1f);
            target_rot_v += v_now *  wall_pidf.getControlVal();
        }
/*
        // 直進時の衝突回避
        if(motion_type == EMotionType::STRAIGHT_WALL_CENTER || 
            motion_type == EMotionType::STRAIGHT) {
            if (WallSensor::getInstance().ahead_l() > pm.wall_diagonal_ahead_l_threshold &&
                WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_r_threshold) {
                target_rot_x -= pm.wall_diagonal_avoid_add_ang;
            } else if (WallSensor::getInstance().ahead_r() > pm.wall_diagonal_ahead_r_threshold &&
                        WallSensor::getInstance().ahead_l() < pm.wall_diagonal_ahead_l_threshold) {
                target_rot_x += pm.wall_diagonal_avoid_add_ang;
            }
        }
*/
        // 斜め直進時の衝突回避
        if(motion_type == EMotionType::DIAGONAL_CENTER) {
            float v_now = constrainL(esti.getV(), 0.1f);
            if (WallSensor::getInstance().ahead_l() < pm.wall_diagonal_ahead_l_threshold) {
                target_rot_v -= v_now * pm.wall_diagonal_avoid_add_ang;
            } else if (WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_r_threshold) {
                target_rot_v += v_now * pm.wall_diagonal_avoid_add_ang;
            }
        }
        // 斜めターン時の衝突回避
/*            
        if(motion_type == EMotionType::CURVE &&
                (( esti.getAng() > 30.0f  && esti.getAng() < 60.0f  ) ||
                    ( esti.getAng() > 120.0f && esti.getAng() < 150.0f ) ||
                    ( esti.getAng() > 210.0f && esti.getAng() < 240.0f ) ||
                    ( esti.getAng() > 300.0f && esti.getAng() < 330.0f )) &&

                (turn_type == turn_type_e::TURN_D_90 ||
                    turn_type == turn_type_e::TURN_D2S_135 ||
                    turn_type == turn_type_e::TURN_S2D_135 ||
                    turn_type == turn_type_e::TURN_D2S_45 ||
                    turn_type == turn_type_e::TURN_S2D_45)


            ) {
            if (WallSensor::getInstance().ahead_l() > 150
                    && WallSensor::getInstance().ahead_l() < pm.wall_diagonal_ahead_l_threshold) {
                target_rot_x -= pm.wall_diagonal_avoid_add_ang;
            } else if (WallSensor::getInstance().ahead_r() > 150 &&
                        WallSensor::getInstance().ahead_r() < pm.wall_diagonal_ahead_l_threshold
                        ) {
                target_rot_x += pm.wall_diagonal_avoid_add_ang;
            }
        }
*/

        ang_pidf.update(target_rot_x, esti.getAng());
        float ang_pidf_controlval = ang_pidf.getControlVal();
        if(wall_pidf.engaged()){
            ang_pidf.reset();
        } 

        target_rot_v += ang_pidf_controlval;


        if( (motion_type != motion_type_pre &&
                (motion_type_pre == EMotionType::STOP || motion_type_pre == EMotionType::SPINTURN)
                )
            || motion_type == EMotionType::DIRECT_DUTY_SET) {
            ang_v_pidf.reset();
            ang_pidf.reset();
            v_pidf.reset();
            wall_pidf.reset();                
        }

        WheelOdometry& wodo = WheelOdometry::getInstance();
        v_pidf.update(target_trans_v, esti.getV());
        ang_v_pidf.update(target_rot_v, esti.getAngV());

        PowerTransmission& pt = PowerTransmission::getInstance();
        duty(0) = 0.0f;
        duty(1) = 0.0f;


        Eigen::Vector2f duty_v_FF(0.0f, 0.0f);
        v_back_emf_FF = pt.transBackEmfDuty(target_trans_v)(0);
        a_acc_FF = pt.transAccDuty(target_trans_a)(0);
        v_fric_FF = pt.transFrictionCompensationDuty(target_trans_v)(0);
        /*
                    duty_v_FF += pt.transAccDuty(target_trans_a);
                    duty_v_FF += pt.transBackEmfDuty(target_trans_v);
                    duty_v_FF += pt.transFrictionCompensationDuty(target_trans_v);
        */
        duty_v_FF += pt.transFFWithParamDuty(target_trans_v, target_trans_a);
        if(pm.trans_v_FF_enable) duty += duty_v_FF;

        Eigen::Vector2f duty_ang_v_FF(0.0f, 0.0f);
        ang_v_back_emf_FF = pt.rotBackEmfDuty(target_rot_v)(0);
        ang_a_acc_FF = pt.rotAccDuty(target_rot_a)(0);
        ang_v_fric_FF = pt.rotFrictionCompensationDuty(target_rot_v)(0);
        /*
                    duty_ang_v_FF += pt.rotAccDuty(target_rot_a);
                    duty_ang_v_FF += pt.rotBackEmfDuty(target_rot_v);
                    duty_ang_v_FF += pt.rotFrictionCompensationDuty(target_rot_v);
        */
        duty_ang_v_FF += pt.rotFFWithParamDuty(target_rot_v, target_rot_a);
        if(pm.rot_v_FF_enable) duty += duty_ang_v_FF;

        float voltage = BatVoltageMonitor::getInstance().bat_vol;
        float duty_v_saturation = (pm.trans_v_saturation_FF_multiplier * ABS(duty_v_FF(0)) + pm.trans_v_saturation_offset_duty);
        float duty_ang_v_saturation = (pm.rot_v_saturation_FF_multiplier * ABS(duty_ang_v_FF(0)) + pm.rot_v_saturation_offset_duty);


        if(motion_type == EMotionType::STOP || motion_type == EMotionType::SPINTURN) {
            // do nothing
        } else {
            v_pidf.setSaturation(duty_v_saturation);
            ang_v_pidf.setSaturation(duty_ang_v_saturation);
        }

        duty(0) += (v_pidf.getControlVal() - ang_v_pidf.getControlVal());
        duty(1) += (v_pidf.getControlVal() + ang_v_pidf.getControlVal());
        // duty飽和時には回転系制御を優先
        if(duty(0) > 1.0 || duty(1) > 1.0) {
            float duty_overflow = 0.0f;
            if(duty(0) > duty(1) ) {
                duty_overflow = duty(0) - 1.0f;
            } else {
                duty_overflow = duty(1) - 1.0f;
            }
            duty(0) -= duty_overflow;
            duty(1) -= duty_overflow;

        }

        motion_type_pre = motion_type;
    
        {
            if( ABS(ang_v_pidf.getError()) > ang_v_error_th ||
                ABS(ang_pidf.getError()) > 5.0f ||
                ABS(v_pidf.getError()) > ABS(v_error_th)
                    ) error_sec += DELTA_T;
            else error_sec = 0.0f;
        }
    


    }

    int usrcmd_controlMixer(int argc, char **argv){
        return 0;
    };
}