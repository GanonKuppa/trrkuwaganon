#pragma once

#include "baseModule.h"

#include <deque>
#include <stdint.h>
#include <Eigen/Geometry>

namespace module {
    class PositionEstimator : public BaseModule<PositionEstimator> {
      public:
        void update0();
        void debug();
        void reset();

      private:
        friend class BaseModule<PositionEstimator>;
                
        PositionEstimator();
        void _publish();
        

        float _v_xy_body;
        float _v_xy_body_enc;
        float _v_xy_body_acc;
        float _v_xy_body_for_odom;

        float _yaw;
        float _roll;
        float _pitch;

        //Eigen::Quaternionf _q;

        float _beta;

        float _yawrate;
        float _rollrate;
        float _pitchrate;

        float _x;
        float _y;
        float _z;

        float _v_x;
        float _v_y;
        float _v_z;

        float _a_body_x;
        float _a_body_y;
        float _a_body_z;

        float _a_x;
        float _a_y;
        float _a_z;

        std::deque<float> _acc_y_list;
        const uint8_t ACC_Y_AVERAGE_NUM = 15;
        const float DEG2RAD = 3.14159265358979f/180.0f;

        const float _afrer_curve_beta_dot_time = 0.050f;
        float _after_curve_time;

    };

    int usrcmd_positionEstimator(int argc, char **argv);
}
