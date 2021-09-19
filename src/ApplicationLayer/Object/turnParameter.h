#pragma once

#include "turnEnum.h"

class TurnParameter {
    public:

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
    { }

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
    { }

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
    { }

    float getV(ETurnType turn_type) {
        switch (turn_type) {
            case ETurnType::STRAIGHT:
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
                return _v_turn_d_90,
            case ETurnType::D_STRAIGHT:
                return _v_d_straight,
            default:
                return 0.0,
        }
    }

    float getAcc(ETurnType turn_type) {
        switch (turn_type) {
            case ETurnType::STRAIGHT:
                return _a_straight,
            case ETurnType::D_STRAIGHT:
                return _a_d_straight,
            default:
                return 0.0,
        }
    }

    private:
    float _v_straight,
    float _v_d_straight,
    float _v_turn_90,
    float _v_turn_l_90;
    float _v_turn_180;
    float _v_turn_d_90;
    float _v_turn_45;
    float _v_turn_135;
    float _a_straight;
    float _a_d_straight;
};

