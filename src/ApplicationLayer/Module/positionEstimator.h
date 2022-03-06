#pragma once



#include "baseModule.h"

#include <deque>
#include <stdint.h>
#include <Eigen/Geometry>
#include "turnEnum.h"
#include "trajTripletMsg.h"
#include "navStateMsg.h"

namespace module {
    class PositionEstimator : public BaseModule<PositionEstimator> {
      public:
        void update0();
        void debug();
        void reset(float x, float y, float yaw);

      private:
        friend class BaseModule<PositionEstimator>;
                
        PositionEstimator();
        void _wallCtrlEngagedCorrection();
        void _onWallCenterCorrection(NavStateMsg &nav_msg);
        void _aheadWallCorrection(bool is_correct_yaw);
        void _cornerLCorrection();
        void _cornerRCorrection();
        void _publish_vehicle_position();
        void _publish_vehicle_attitude();
        void _aheadWallCorrectionOnWallRead(float dist_a);


        void _edgeLCorrection(TrajTripletMsg &traj_msg);
        void _edgeRCorrection(TrajTripletMsg &traj_msg);
        void _diagEdgeLCorrection(TrajTripletMsg &traj_msg);
        void _diagEdgeRCorrection(TrajTripletMsg &traj_msg);
        
        float _x;
        float _y;
        float _z;

        float _v_xy_body_cmp;
        float _v_xy_body_enc;
        float _v_xy_body_ave;
        float _v_xy_body_acc;
        
        float _v_x;
        float _v_y;
        float _v_z;

        float _v_xy_body_for_odom;
        float _v_xy_body_for_ctrl;

        float _a_body_x;
        float _a_body_y;
        float _a_body_z;

        float _yaw;
        float _roll;
        float _pitch;

        Eigen::Quaternionf _q;

        float _roll_acc;
        float _pitch_acc;

        float _beta;
        float _beta_dot;

        float _yawrate;
        float _rollrate;
        float _pitchrate;

        float _yaw_error_accum;

        ETurnType _turn_type;

        std::deque<float> _acc_y_list;
        std::deque<float> _v_enc_list;
        const uint8_t ACC_Y_AVERAGE_NUM = 10;
        const float PI = 3.14159265f;
        const float DEG2RAD = PI/180.0f;
        const float RAD2DEG = 180.0f/PI;

        const float _afrer_curve_beta_expiration_time = 0.050f;
        float _beta_expiration_time;
        float _on_wall_center_dist;
        float _corner_r_cool_down_dist;
        float _corner_l_cool_down_dist;

        bool _in_read_wall_area_pre;
        bool _detected_edge;

    };

    int usrcmd_positionEstimator(int argc, char **argv);
}
