#pragma once

#include "turnEnum.h"

// Lib
#include "debugLog.h"

class TurnParameter {
  public:

    TurnParameter() :
    _v_straight(0.3f),
    _v_d_straight(0.3f),
    _v_turn_90(0.3f),
    _v_turn_l_90(0.3f),
    _v_turn_180(0.3f),
    _v_turn_d_90(0.3f),
    _v_turn_45(0.3f),
    _v_turn_135(0.3f),
    _a_straight(2.0f),
    _a_d_straight(2.0f)
    { };

    TurnParameter(float v_straight, float v_d_straight, float v_turn_90, float v_turn_l_90, float v_turn_180,
                    float v_turn_d_90, float v_turn_45, float v_turn_135, float a_straight, float a_d_straight) :
    _v_straight(v_straight),
    _v_d_straight(v_d_straight),
    _v_turn_90(v_turn_90),
    _v_turn_l_90(v_turn_l_90),
    _v_turn_180(v_turn_180),
    _v_turn_d_90(v_turn_d_90),
    _v_turn_45(v_turn_45),
    _v_turn_135(v_turn_135),
    _a_straight(a_straight),
    _a_d_straight(a_d_straight)
    { };

    TurnParameter(float v, float a) :
    _v_straight(v),
    _v_d_straight(v),
    _v_turn_90(v),
    _v_turn_l_90(v),
    _v_turn_180(v),
    _v_turn_d_90(v),
    _v_turn_45(v),
    _v_turn_135(v),
    _a_straight(a),
    _a_d_straight(a)
    { };

    TurnParameter(float v, float v_d, float turn_v, float a) :
    _v_straight(v),
    _v_d_straight(v_d),
    _v_turn_90(turn_v),
    _v_turn_l_90(turn_v),
    _v_turn_180(turn_v),
    _v_turn_d_90(turn_v),
    _v_turn_45(turn_v),
    _v_turn_135(turn_v),
    _a_straight(a),
    _a_d_straight(a)
    { };

    float getV(ETurnType turn_type) {
        switch (turn_type) {
            case ETurnType::STRAIGHT:
                return _v_straight;
            case ETurnType::STRAIGHT_WALL_CENTER:
            	return _v_straight;
            case ETurnType::TURN_90:
                return _v_turn_90;
            case ETurnType::TURN_L_90:
                return _v_turn_l_90;
            case ETurnType::TURN_180:
                return _v_turn_180;
            case ETurnType::TURN_S2D_45:
                return _v_turn_45;
            case ETurnType::TURN_S2D_135:
                return _v_turn_135;
            case ETurnType::TURN_D2S_45:
                return _v_turn_45;
            case ETurnType::TURN_D2S_135:
                return _v_turn_135;
            case ETurnType::TURN_D_90:
                return _v_turn_d_90;
            case ETurnType::DIAGONAL:
                return _v_d_straight;
            case ETurnType::DIAGONAL_CENTER:
                return _v_d_straight;
            default:
                return 0.0;
        }
    };

    float getAcc(ETurnType turn_type) {
        switch (turn_type) {
            case ETurnType::STRAIGHT:
                return _a_straight;
            case ETurnType::STRAIGHT_WALL_CENTER:
            	return _a_straight;
            case ETurnType::DIAGONAL:
                return _a_d_straight;
            case ETurnType::DIAGONAL_CENTER:
                return _a_d_straight;
            default:
                return 0.0;
        }
    };

    void print() {
        PRINTF_ASYNC("    v_straight   | %f \n", _v_straight);
        PRINTF_ASYNC("    v_d_straight | %f \n", _v_d_straight);
        PRINTF_ASYNC("    v_turn_90    | %f \n", _v_turn_90);
        PRINTF_ASYNC("    v_turn_l_90  | %f \n", _v_turn_l_90);
        PRINTF_ASYNC("    v_turn_180   | %f \n", _v_turn_180);
        PRINTF_ASYNC("    v_turn_d_90  | %f \n", _v_turn_d_90);
        PRINTF_ASYNC("    v_turn_45    | %f \n", _v_turn_45);
        PRINTF_ASYNC("    v_turn_135   | %f \n", _v_turn_135);
        PRINTF_ASYNC("    a_straight   | %f \n", _a_straight);
        PRINTF_ASYNC("    a_d_straight | %f \n", _a_d_straight);
    };

  private:
    float _v_straight;
    float _v_d_straight;
    float _v_turn_90;
    float _v_turn_l_90;
    float _v_turn_180;
    float _v_turn_d_90;
    float _v_turn_45;
    float _v_turn_135;
    float _a_straight;
    float _a_d_straight;
};

