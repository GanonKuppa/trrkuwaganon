#pragma once

#include "BaseMsg.h"

class PositionEstimatorMsg : public BaseMsg{
  public:
    float v_xy_body = 0.0f; // m/s
    float v_xy_body_enc = 0.0f; // m/s
    float v_xy_body_acc = 0.0f; // m/s
    float v_xy_body_for_odom = 0.0f; // m/s

    float yaw = 0.0f; // rad
    float roll = 0.0f; // rad
    float pitch = 0.0f; // rad

    float q0 = 1.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;

    float roll_acc = 0.0f; // rad
    float pitch_acc = 0.0f; // rad

    float beta = 0.0f; // rad

    float yawrate = 0.0f; // rad/s
    float rollrate = 0.0f; // rad/s
    float pitchrate = 0.0f; // rad/s

    float x = 0.0f; // m
    float y = 0.0f; // m
    float z = 0.0f; // m

    float v_x = 0.0f; // m/s
    float v_y = 0.0f; // m/s
    float v_z = 0.0f; // m/s

    float a_body_x = 0.0f; // m/s^2
    float a_body_y = 0.0f; // m/s^2
    float a_body_z = 0.0f; // m/s^2

    float a_x = 0.0f; // m/s^2
    float a_y = 0.0f; // m/s^2
    float a_z = 0.0f; // m/s^2

    float _after_curve_time = 0.0f; //sec
};