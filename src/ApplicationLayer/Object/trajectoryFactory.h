#pragma once

#include <memory>

#include "trajectory.h"


class StraightFactory{
  public: 
    static std::unique_ptr<BaseTrajectory> create(ETurnType turn_type, float target_dist, float v_0);
    static std::unique_ptr<BaseTrajectory> create(ETurnType turn_type, float target_dist, float v_0, float v_max, float v_end, float a_acc, float a_dec);
    static void push(ETurnType turn_type, float target_dist, float v_0);
    static void push(ETurnType turn_type, float target_dist, float v_0, float v_max, float v_end, float a_acc, float a_dec);
};

class SpinTurnFactory{
  public:
    static std::unique_ptr<BaseTrajectory> create(float target_cumulative_yaw, float abs_yawrate_max, float abs_yawacc);
    static void push(float target_cumulative_yaw, float abs_yawrate_max, float abs_yawacc);
};

class StopFactory{
  public:  
    static std::unique_ptr<BaseTrajectory> create(float stop_time);
    static void push(float stop_time);
};

class CurveFactory{
  public:  
    static std::unique_ptr<BaseTrajectory> create(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir);
    static void push(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir);
    static void pushWithStraight(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir);
    static void pushWithStraight(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir, float offset_pre, float offset_fol);
};

