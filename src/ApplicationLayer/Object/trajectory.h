#pragma once

#include <memory>

#include "turnEnum.h"
#include "turnIterator.h"


class BaseTrajectory {
  public:
    BaseTrajectory();
    void setInitPos(float x, float y, float yaw);
    virtual float getEndX() = 0;
    virtual float getEndY() = 0;
    virtual float getEndYaw() = 0;
    virtual float getNecessaryTime();
    virtual void update();
    virtual bool isEnd()=0;

    void publish();

    virtual ~BaseTrajectory() { }

  protected:
    float _x;
    float _v_x;
    float _a_x;
    float _y;
    float _v_y;
    float _a_y;
    float _v_xy_body;
    float _a_xy_body;
    float _yaw;
    float _yawrate;
    float _yawacc;
    float _beta;
    float _beta_dot;    
    float _delta_t;
    float _x_0;
    float _y_0;
    float _yaw_0;
    float _cumulative_dist;
    float _cumulative_yaw;
    float _cumulative_t;

    bool _in_detect_edge_area;

    ETrajType _traj_type;
    ETurnType _turn_type;
    ETurnDir _turn_dir;
};


class StraightTrajectory : public BaseTrajectory {
  public:
    StraightTrajectory(ETurnType turn_type, float target_dist, float v_0);
    StraightTrajectory(ETurnType turn_type, float target_dist, float v_0, float v_max, float v_end, float a_acc, float a_dec);
    virtual float getEndX();    
    virtual float getEndY();
    virtual float getEndYaw();
    virtual float getNecessaryTime();    
    virtual void update();    
    virtual bool isEnd();

    virtual ~StraightTrajectory() {}

  private:
    float _v_min;
    float _v_max;
    float _v_end;
    float _a_acc;
    float _a_dec;
    float _v_0;
    float _target_dist;

    bool _detected_edge;    

    float _calcResidualDist(float x_esti, float y_esti);
};


class SpinTurnTrajectory : public BaseTrajectory {
  public:
    SpinTurnTrajectory(float target_cumulative_yaw, float abs_yawrate_max, float abs_yawacc);
    virtual float getEndX();
    virtual float getEndY();
    virtual float getEndYaw();
    virtual void update();
    virtual bool isEnd();

    virtual ~SpinTurnTrajectory() {}
  private:
    float _target_cumulative_yaw;
    float _abs_yawacc;
    float _abs_yawrate_max;
    float _abs_yawrate_min;
};

class StopTrajectory : public BaseTrajectory {
  public:
    StopTrajectory(float stop_time);
    StopTrajectory(float stop_time, float x, float y, float yaw);

    virtual float getEndX();
    virtual float getEndY();
    virtual float getEndYaw();
    virtual float getNecessaryTime();

    virtual void update();
    virtual bool isEnd();

    virtual ~StopTrajectory() {}
  private:
    float _stop_time;
};

class CurveTrajectory : public BaseTrajectory {
  public:    
    CurveTrajectory(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir);
    virtual float getEndX();
    virtual float getEndY();
    virtual float getEndYaw();
    virtual float getNecessaryTime();
    virtual void update();
    virtual bool isEnd();

    virtual ~CurveTrajectory() {}
  private:
    std::unique_ptr<TurnIterator> _turn_iterator;
    float _yaw_pre;
    float _beta_pre;
    float _yawrate_pre;
    ETurnParamSet _param_set;
};

class AheadWallCorrectionTrajectory : public BaseTrajectory {
  public:
    AheadWallCorrectionTrajectory(float stop_time, bool is_yaw_correct = false);
    AheadWallCorrectionTrajectory(float stop_time, float calm_time, bool is_yaw_correct = false);
    

    virtual float getEndX();
    virtual float getEndY();
    virtual float getEndYaw();
    virtual float getNecessaryTime();

    virtual void update();
    virtual bool isEnd();

    virtual ~AheadWallCorrectionTrajectory() {}
  private:
    float _stop_time;
    float _calm_time;
    float _cumulative_calm_t;
};
