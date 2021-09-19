#pragma once

#include <memory>

class TurnPreCalculation{
  public:  
    TurnPreCalculation();
    std::unique_ptr<TurnIterator> GenerateTurnIterator();

    void init();
    float getTrajEndTime();
    float getPreDist();
    float getFolDist();
  private:
    float _pre_dist;
    float _fol_dist;
    float _shape_factor;
    float _path_length;
    float _target_ang        
    float _start_ang; 
    float _start_x;
    float _start_y;
    float _int_molli_c;
    float _v;
    float _cp;
}

class TurnPreCalculations{
  public:
    TurnPreCalculation turn_90;
    TurnPreCalculation turn_l_90;
    TurnPreCalculation turn_180;
    TurnPreCalculation turn_s2d_45;
    TurnPreCalculation turn_s2d_135;
    TurnPreCalculation turn_d_90;
    TurnPreCalculation turn_d2s_45;
    TurnPreCalculation turn_d2s_135;

    float getTrajEndTime(ETurnType tt){
        switch (tt) {
            case ETurnType::TURN_90:
                return turn_90.getTrajEndTime();
            case ETurnType::TURN_L_90:
                return turn_l_90.getTrajEndTime();
            case ETurnType::TURN_180:
                return turn_180.getTrajEndTime();
            case ETurnType::TURN_S2D_45:
                return turn_45.getTrajEndTime();
            case ETurnType::TURN_S2D_135:
                return turn_135.getTrajEndTime();
            case ETurnType::TURN_D2S_45:
                return turn_45.getTrajEndTime();
            case ETurnType::TURN_D2S_135:
                return turn_135.getTrajEndTime();
            case ETurnType::TURN_D_90:
                return turn_d_90.getTrajEndTime(),
            default:
                return 0.0,
        }
    };
    float getPreDist(ETurnType tt){
        switch (tt) {
            case ETurnType::TURN_90:
                return turn_90.getPreDist();
            case ETurnType::TURN_L_90:
                return turn_l_90.getPreDist();
            case ETurnType::TURN_180:
                return turn_180.getPreDist();
            case ETurnType::TURN_S2D_45:
                return turn_45.getPreDist();
            case ETurnType::TURN_S2D_135:
                return turn_135.getPreDist();
            case ETurnType::TURN_D2S_45:
                return turn_45.getPreDist();
            case ETurnType::TURN_D2S_135:
                return turn_135.getPreDist();
            case ETurnType::TURN_D_90:
                return turn_d_90.getPreDist(),
            default:
                return 0.0,
        }
    };
    float getFolDist(ETurnType tt){
        switch (tt) {
            case ETurnType::TURN_90:
                return turn_90.getFolDist();
            case ETurnType::TURN_L_90:
                return turn_l_90.getFolDist();
            case ETurnType::TURN_180:
                return turn_180.getFolDist();
            case ETurnType::TURN_S2D_45:
                return turn_45.getFolDist();
            case ETurnType::TURN_S2D_135:
                return turn_135.getFolDist();
            case ETurnType::TURN_D2S_45:
                return turn_45.getFolDist();
            case ETurnType::TURN_D2S_135:
                return turn_135.getFolDist();
            case ETurnType::TURN_D_90:
                return turn_d_90.getFolDist();
            default:
                return 0.0,
        }

    };
    std::unique_ptr<TurnIterator> generateTurnIterator(tt){
        switch (tt) {
            case ETurnType::TURN_90:
                return turn_90.generateTurnIterator();
            case ETurnType::TURN_L_90:
                return turn_l_90.generateTurnIterator();
            case ETurnType::TURN_180:
                return turn_180.generateTurnIterator();
            case ETurnType::TURN_S2D_45:
                return turn_45.generateTurnIterator();
            case ETurnType::TURN_S2D_135:
                return turn_135.generateTurnIterator();
            case ETurnType::TURN_D2S_45:
                return turn_45.generateTurnIterator();
            case ETurnType::TURN_D2S_135:
                return turn_135.generateTurnIterator();
            case ETurnType::TURN_D_90:
                return turn_d_90.generateTurnIterator();
            default:
                return nullptr;
        }
    };



}

