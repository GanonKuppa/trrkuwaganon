#include "turnPreCalculations.h"

    constexpr float DELTA_T = 0.001f;
    constexpr float DEG2RAD = 3.1415926535f / 180.0f;

    constexpr float SHAPE_FACTOR_90 = 4.0f;
    constexpr float SHAPE_FACTOR_L_90 = 4.0f;
    constexpr float SHAPE_FACTOR_180 = 3.6f;
    constexpr float SHAPE_FACTOR_S2D_45 = 4.0f;
    constexpr float SHAPE_FACTOR_S2D_135 = 4.0f;
    constexpr float SHAPE_FACTOR_D_90 = 4.0f;
    constexpr float SHAPE_FACTOR_D2S_45 = 4.0f;
    constexpr float SHAPE_FACTOR_D2S_135 = 4.0f;

    constexpr float PATH_LENGTH_90 = 0.06f;
    constexpr float PATH_LENGTH_L_90 = 0.11f;
    constexpr float PATH_LENGTH_180 = 0.1f;
    constexpr float PATH_LENGTH_S2D_45 = 0.04f;
    constexpr float PATH_LENGTH_S2D_135 = 0.11f;
    constexpr float PATH_LENGTH_D_90 = 0.07f;
    constexpr float PATH_LENGTH_D2S_45 = 0.04f;
    constexpr float PATH_LENGTH_D2S_135 = 0.11f;

    constexpr float TARGET_ANG_90 = 90.0f * DEG2RAD;
    constexpr float TARGET_ANG_L_90 = 90.0f * DEG2RAD;
    constexpr float TARGET_ANG_180 = 180.0f * DEG2RAD;
    constexpr float TARGET_ANG_S2D_45 = 45.0f * DEG2RAD;
    constexpr float TARGET_ANG_S2D_135 = 135.0f * DEG2RAD;
    constexpr float TARGET_ANG_D_90 = 90.0f * DEG2RAD;
    constexpr float TARGET_ANG_D2S_45 = 45.0f * DEG2RAD;
    constexpr float TARGET_ANG_D2S_135 = 135.0f * DEG2RAD;

    constexpr float START_ANG_90 = 0.0f * DEG2RAD;
    constexpr float START_ANG_L_90 = 0.0f * DEG2RAD;
    constexpr float START_ANG_180 = 0.0f * DEG2RAD;
    constexpr float START_ANG_S2D_45 = 0.0f * DEG2RAD;
    constexpr float START_ANG_S2D_135 = 0.0f * DEG2RAD;
    constexpr float START_ANG_D_90 = 45.0f * DEG2RAD;
    constexpr float START_ANG_D2S_45 = 45.0f * DEG2RAD;
    constexpr float START_ANG_D2S_135 = 45.0f * DEG2RAD;

    constexpr float START_X_90 = -0.045f;
    constexpr float START_X_L_90 = -0.09f;
    constexpr float START_X_180 = 0.0f;
    constexpr float START_X_S2D_45 = -0.09f;
    constexpr float START_X_S2D_135 = -0.09f;
    constexpr float START_X_D_90 = -0.045f;
    constexpr float START_X_D2S_45 = -0.045f;
    constexpr float START_X_D2S_135 = -0.045f;

    constexpr float START_Y_90 = 0.0f;
    constexpr float START_Y_L_90 = 0.0f;
    constexpr float START_Y_180 = 0.0f;
    constexpr float START_Y_S2D_45 = 0.0f;
    constexpr float START_Y_S2D_135 = 0.0f;
    constexpr float START_Y_D_90 = 0.0f;
    constexpr float START_Y_D2S_45 = 0.0f;
    constexpr float START_Y_D2S_135 = 0.0f;

    constexpr float END_X_90      = 0.0f;
    constexpr float END_X_L_90    = 0.0f;
    constexpr float END_X_180     = 0.0f;
    constexpr float END_X_S2D_45  = 0.0f;
    constexpr float END_X_S2D_135 = -0.045f;
    constexpr float END_X_D_90    = -0.045f;
    constexpr float END_X_D2S_45  = 0.0f;
    constexpr float END_X_D2S_135 = -0.090f;

    constexpr float END_Y_90      = 0.045f;
    constexpr float END_Y_L_90    = 0.090f;
    constexpr float END_Y_180     = 0.090f;
    constexpr float END_Y_S2D_45  = 0.045f;
    constexpr float END_Y_S2D_135 = 0.090f;
    constexpr float END_Y_D_90    = 0.090f;
    constexpr float END_Y_D2S_45  = 0.090f;
    constexpr float END_Y_D2S_135 = 0.090f;



TurnPreCalculations::TurnPreCalculations(float cp, TurnParameter tp){
    _cp = cp;
    _tp = tp;
    _init();
}

TurnPreCalculations::TurnPreCalculations(){
    _cp = 100.0f;
    _tp = TurnParameter();
    _init();
}


float TurnPreCalculations::getTrajEndTime(ETurnType tt){
    switch (tt) {
        case ETurnType::TURN_90:
            return _turn_90.getTrajEndTime();
        case ETurnType::TURN_L_90:
            return _turn_l_90.getTrajEndTime();
        case ETurnType::TURN_180:
            return _turn_180.getTrajEndTime();
        case ETurnType::TURN_S2D_45:
            return _turn_s2d_45.getTrajEndTime();
        case ETurnType::TURN_S2D_135:
            return _turn_s2d_135.getTrajEndTime();
        case ETurnType::TURN_D2S_45:
            return _turn_d2s_45.getTrajEndTime();
        case ETurnType::TURN_D2S_135:
            return _turn_d2s_135.getTrajEndTime();
        case ETurnType::TURN_D_90:
            return _turn_d_90.getTrajEndTime();
        default:
            return 0.0;
    }
}

float TurnPreCalculations::getPreDist(ETurnType tt){
    switch (tt) {
        case ETurnType::TURN_90:
            return _turn_90.getPreDist();
        case ETurnType::TURN_L_90:
            return _turn_l_90.getPreDist();
        case ETurnType::TURN_180:
            return _turn_180.getPreDist();
        case ETurnType::TURN_S2D_45:
            return _turn_s2d_45.getPreDist();
        case ETurnType::TURN_S2D_135:
            return _turn_s2d_135.getPreDist();
        case ETurnType::TURN_D2S_45:
            return _turn_d2s_45.getPreDist();
        case ETurnType::TURN_D2S_135:
            return _turn_d2s_135.getPreDist();
        case ETurnType::TURN_D_90:
            return _turn_d_90.getPreDist();
        default:
            return 0.0;
    }
}

float TurnPreCalculations::getFolDist(ETurnType tt){
    switch (tt) {
        case ETurnType::TURN_90:
            return _turn_90.getFolDist();
        case ETurnType::TURN_L_90:
            return _turn_l_90.getFolDist();
        case ETurnType::TURN_180:
            return _turn_180.getFolDist();
        case ETurnType::TURN_S2D_45:
            return _turn_s2d_45.getFolDist();
        case ETurnType::TURN_S2D_135:
            return _turn_s2d_135.getFolDist();
        case ETurnType::TURN_D2S_45:
            return _turn_d2s_45.getFolDist();
        case ETurnType::TURN_D2S_135:
            return _turn_d2s_135.getFolDist();
        case ETurnType::TURN_D_90:
            return _turn_d_90.getFolDist();
        default:
            return 0.0;
    }
}

std::unique_ptr<TurnIterator> TurnPreCalculations::generateTurnIterator(ETurnType tt){
    switch (tt) {
        case ETurnType::TURN_90:
            return _turn_90.generateTurnIterator();
        case ETurnType::TURN_L_90:
            return _turn_l_90.generateTurnIterator();
        case ETurnType::TURN_180:
            return _turn_180.generateTurnIterator();
        case ETurnType::TURN_S2D_45:
            return _turn_s2d_45.generateTurnIterator();
        case ETurnType::TURN_S2D_135:
            return _turn_s2d_135.generateTurnIterator();
        case ETurnType::TURN_D2S_45:
            return _turn_d2s_45.generateTurnIterator();
        case ETurnType::TURN_D2S_135:
            return _turn_d2s_135.generateTurnIterator();
        case ETurnType::TURN_D_90:
            return _turn_d_90.generateTurnIterator();
        default:
            return nullptr;
    }
}

void TurnPreCalculations::_init(){
    return;
    _turn_90 = TurnPreCalculation(
        SHAPE_FACTOR_90, 
        PATH_LENGTH_90,
        TARGET_ANG_90,
        START_ANG_90,
        START_X_90,
        START_Y_90,
        END_X_90,
        END_Y_90,
        _tp.getV(ETurnType::TURN_90),
        _cp,
        DELTA_T
    );
    
    _turn_l_90 = TurnPreCalculation(
        SHAPE_FACTOR_L_90, 
        PATH_LENGTH_L_90,
        TARGET_ANG_L_90,
        START_ANG_L_90,
        START_X_L_90,
        START_Y_L_90,
        END_X_L_90,
        END_Y_L_90,
        _tp.getV(ETurnType::TURN_L_90),
        _cp,
        DELTA_T
    );

    _turn_180 = TurnPreCalculation(
        SHAPE_FACTOR_180, 
        PATH_LENGTH_180,
        TARGET_ANG_180,
        START_ANG_180,
        START_X_180,
        START_Y_180,
        END_X_180,
        END_Y_180,
        _tp.getV(ETurnType::TURN_180),
        _cp,
        DELTA_T
    );

    _turn_s2d_45 = TurnPreCalculation(
        SHAPE_FACTOR_S2D_45, 
        PATH_LENGTH_S2D_45,
        TARGET_ANG_S2D_45,
        START_ANG_S2D_45,
        START_X_S2D_45,
        START_Y_S2D_45,
        END_X_S2D_45,
        END_Y_S2D_45,
        _tp.getV(ETurnType::TURN_S2D_45),
        _cp,
        DELTA_T
    );

    _turn_s2d_135 = TurnPreCalculation(
        SHAPE_FACTOR_S2D_135, 
        PATH_LENGTH_S2D_135,
        TARGET_ANG_S2D_135,
        START_ANG_S2D_135,
        START_X_S2D_135,
        START_Y_S2D_135,
        END_X_S2D_135,
        END_Y_S2D_135,
        _tp.getV(ETurnType::TURN_S2D_135),
        _cp,
        DELTA_T
    );

    _turn_d_90 = TurnPreCalculation(
        SHAPE_FACTOR_D_90, 
        PATH_LENGTH_D_90,
        TARGET_ANG_D_90,
        START_ANG_D_90,
        START_X_D_90,
        START_Y_D_90,
        END_X_D_90,
        END_Y_D_90,
        _tp.getV(ETurnType::TURN_D_90),
        _cp,
        DELTA_T
    );

    _turn_d2s_45 = TurnPreCalculation(
        SHAPE_FACTOR_D2S_45, 
        PATH_LENGTH_D2S_45,
        TARGET_ANG_D2S_45,
        START_ANG_D2S_45,
        START_X_D2S_45,
        START_Y_D2S_45,
        END_X_D2S_45,
        END_Y_D2S_45,
        _tp.getV(ETurnType::TURN_D2S_45),
        _cp,
        DELTA_T
    );

    _turn_d2s_135 = TurnPreCalculation(
        SHAPE_FACTOR_D2S_135, 
        PATH_LENGTH_D2S_135,
        TARGET_ANG_D2S_135,
        START_ANG_D2S_135,
        START_X_D2S_135,
        START_Y_D2S_135,
        END_X_D2S_135,
        END_Y_D2S_135,
        _tp.getV(ETurnType::TURN_D2S_135),
        _cp,
        DELTA_T
    );
}
