#pragma once

#include <memory>


#include "turnParameter.h"
#include "turnEnum.h"
#include "turnIterator.h"
#include "turnPreCalculation.h"

class TurnPreCalculations{
  public:
    TurnPreCalculations();
    TurnPreCalculations(float cp, TurnParameter tp);
    TurnPreCalculations(const TurnPreCalculations&) = default;
    TurnPreCalculations& operator = (const TurnPreCalculations&) = default;


    float getTrajEndTime(ETurnType tt);
    float getPreDist(ETurnType tt);
    float getFolDist(ETurnType tt);
    float getMoveX(ETurnType tt);
    float getMoveY(ETurnType tt);
    std::unique_ptr<TurnIterator> generateTurnIterator(ETurnType tt);
  
  private:    
    float _cp;
    TurnParameter _tp;

    TurnPreCalculation _turn_90;
    TurnPreCalculation _turn_l_90;
    TurnPreCalculation _turn_180;
    TurnPreCalculation _turn_s2d_45;
    TurnPreCalculation _turn_s2d_135;
    TurnPreCalculation _turn_d_90;
    TurnPreCalculation _turn_d2s_45;
    TurnPreCalculation _turn_d2s_135;

    void _init();
};
