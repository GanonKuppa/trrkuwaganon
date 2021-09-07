#pragma once

#include "baseModule.h"
#include <stdint.h>

// Lib
#include "pidController.h"

// Msg
#include "ctrlSetpointMsg.h"
#include "navStateMsg.h"
#include "wallSensorMsg.h"
#include "vehicleAttitudeMsg.h"
#include "vehiclePositionMsg.h"

namespace module {
    class ControlMixer : public BaseModule<ControlMixer> {
        void update0();
      private:
        friend class BaseModule<ControlMixer>;
        
        WallPidfController _wall_pidf;
        PidfController _wall_diag_pidf;
        AngPidfController _yaw_pidf;
        PidfController _yawrate_pidf;
        PidfController _v_pidf;

        CtrlSetpointMsg _setp_msg;
        NavStateMsg _nav_msg;
        WallSensorMsg _ws_msg;
        VehicleAttitudeMsg _att_msg;
        VehiclePositionMsg _pos_msg;

        bool _isLWall;
        bool _isRWall;
        float _center_dist_r;
        float _center_dist_l;
        float _ahead_dist_r;
        float _ahead_dist_l;

        float _setp_v_xy_body;
        float _setp_a_xy_body;
        float _setp_yaw;
        float _setp_yawrate;
        float _setp_yawacc;
        
        ETrajType _traj_type_pre;
        ETrajType _traj_type;
        ETurnType _turn_type_pre;
        ETurnType _turn_type;

        float _duty_r;
        float _duty_l;

        float _duty_r_v;
        float _duty_l_v;

        float _duty_r_yaw;
        float _duty_l_yaw;

        float _error_sec;
        bool _is_error;
        static constexpr float YAWRATE_ERROR_TH = 0.26f; // 0.26 rad/s = 15 deg/s  
        static constexpr float V_ERROR_TH = 0.2f; // m/s
        static constexpr float ERROR_TIME_TH = 0.5f;
        ControlMixer();

        void _updateControllerParam();
        void _msgUpdate();
        void _updateController();
        void _publish();

        float _sign(float val);
    };

    int usrcmd_controlMixer(int argc, char **argv);
}
