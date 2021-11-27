#include "trajectoryInitializer.h"

#include <stdint.h>
#include <utility>

// Lib
#include "ntlibc.h"
#include "debugLog.h"
#include "mollifier.h"


// Hal
#include "hal_timer.h"


// Module
#include "parameterManager.h"

namespace module {

    TrajectoryInitializer::TrajectoryInitializer(){
        setModuleName("TrajectoryInitializer");
        _init();
    }

    float TrajectoryInitializer::getV(ETurnParamSet tp, ETurnType tt){
        return _turnParamSet.at(tp).getV(tt);
    }

    float TrajectoryInitializer::getAcc(ETurnParamSet tp, ETurnType tt){
        return _turnParamSet.at(tp).getAcc(tt);
    }
    
    float TrajectoryInitializer::getPreDist(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getPreDist(tt);
    }

    float TrajectoryInitializer::getFolDist(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getFolDist(tt);
    }

    float TrajectoryInitializer::getMoveX(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getMoveX(tt);
    }

    float TrajectoryInitializer::getMoveY(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getMoveY(tt);
    }

    float TrajectoryInitializer::getTrajEndTime(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getTrajEndTime(tt);
    }

    std::unique_ptr<TurnIterator> TrajectoryInitializer::generateTurnIterator(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).generateTurnIterator(tt);
    }

    TurnParameter TrajectoryInitializer::getTurnParameter(ETurnParamSet tp){
        return _turnParamSet.at(tp);
    }

    void TrajectoryInitializer::_init(){
        ParameterManager& pm = ParameterManager::getInstance();
        _turnParamSet[ETurnParamSet::SEARCH] = TurnParameter(pm.v_search_run, pm.a_search_run);
        _turnParamSet[ETurnParamSet::SAFE0] = TurnParameter(0.4f, 3.0f);
        _turnParamSet[ETurnParamSet::SAFE1] = TurnParameter(1.0f, 0.5f, 0.5f, 4.0f);
        _turnParamSet[ETurnParamSet::FAST0] = TurnParameter(
                pm.shortest_0_v, 
                pm.shortest_0_v_d,
                pm.shortest_0_v_90,
                pm.shortest_0_v_l90,
                pm.shortest_0_v_180,
                pm.shortest_0_v_d90,
                pm.shortest_0_v_45,
                pm.shortest_0_v_135,
                pm.shortest_0_a,
                pm.shortest_0_a_diag
        );
        _turnParamSet[ETurnParamSet::FAST1] = TurnParameter(
                pm.shortest_1_v,
                pm.shortest_1_v_d,
                pm.shortest_1_v_90,
                pm.shortest_1_v_l90,
                pm.shortest_1_v_180,
                pm.shortest_1_v_d90,
                pm.shortest_1_v_45,
                pm.shortest_1_v_135,
                pm.shortest_1_a,
                pm.shortest_1_a_diag
        );
        _turnParamSet[ETurnParamSet::FAST2] = TurnParameter(
                pm.shortest_2_v,
                pm.shortest_2_v_d,
                pm.shortest_2_v_90,
                pm.shortest_2_v_l90,
                pm.shortest_2_v_180,
                pm.shortest_2_v_d90,
                pm.shortest_2_v_45,
                pm.shortest_2_v_135,
                pm.shortest_2_a,
                pm.shortest_2_a_diag
        );
        _turnParamSet[ETurnParamSet::FAST3] = TurnParameter(
                pm.shortest_3_v,
                pm.shortest_3_v_d,
                pm.shortest_3_v_90,
                pm.shortest_3_v_l90,
                pm.shortest_3_v_180,
                pm.shortest_3_v_d90,
                pm.shortest_3_v_45,
                pm.shortest_3_v_135,
                pm.shortest_3_a,
                pm.shortest_3_a_diag
        );
        _turnParamSet[ETurnParamSet::FAST4] = TurnParameter(
                pm.shortest_4_v,
                pm.shortest_4_v_d,
                pm.shortest_4_v_90,
                pm.shortest_4_v_l90,
                pm.shortest_4_v_180,
                pm.shortest_4_v_d90,
                pm.shortest_4_v_45,
                pm.shortest_4_v_135,
                pm.shortest_4_a,
                pm.shortest_4_a_diag
        );
        
        float cp = pm.cp_coef;                
        for (std::pair<ETurnParamSet, TurnParameter> key_val : _turnParamSet){
            _turnPreCalcs[key_val.first] = TurnPreCalculations(cp, key_val.second);
        }

    }


    void TrajectoryInitializer::debugMollifier(){
        
        PRINTF_ASYNC("--- time measure mollifier_f() --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            for(uint32_t i=0; i<100000; i++){
                mollifier_f(0.5f, 1.0f, 1.0f, 2.0f);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  100000 times [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave        [ms] |  %f \n", (float)(end_time - start_time) / 100000.0f );
        }

        PRINTF_ASYNC("--- time measure intMollifier_f() dt = 0.0001 --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            
            float c = 0.1f;
            for(uint16_t i=1; i<=40; i++){
                c = 0.1f * (float)i;
                float S = intMollifier_f(c, 0.0001);
                PRINTF_ASYNC("  %f, %f\n", c, S);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  40 times [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave      [ms] |  %f \n", (float)(end_time - start_time) / 40.0f );
        }

        PRINTF_ASYNC("--- time measure intMollifier()_f dt = 0.0005 --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            
            float c = 0.1f;
            for(uint16_t i=1; i<=40; i++){
                c = 0.1f * (float)i;
                float S = intMollifier_f(c, 0.0005);
                PRINTF_ASYNC("  %f, %f\n", c, S);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  40 times [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave      [ms] |  %f \n", (float)(end_time - start_time) / 40.0f );
        }

        PRINTF_ASYNC("--- time measure intMollifier()_f_tbl --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            
            float c = 0.1f;
            for(uint16_t i=1; i<=40; i++){
                c = 0.1f * (float)i;
                float S = intMollifier_f_tbl(c);
                PRINTF_ASYNC("  %f, %f\n", c, S);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  40 times [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave      [ms] |  %f \n", (float)(end_time - start_time) / 40.0f );
        }

        PRINTF_ASYNC("--- time measure mollifire_d() --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            for(uint16_t i=0; i<100; i++){
                mollifier_d(0.5, 1.0, 1.0, 2.0);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  100 times  [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave        [ms] |  %f \n", (float)(end_time - start_time) / 100.0f );
        }

        PRINTF_ASYNC("--- time measure intMollifier_d() dt = 0.001 --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            
            double c = 0.1;
            for(uint16_t i=1; i<=40; i++){
                c = 0.1 * (double)i;
                double S = intMollifier_d(c, 0.001);
                PRINTF_ASYNC("  %f, %f\n", c, S);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  40 times [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave      [ms] |  %f \n", (float)(end_time - start_time) / 40.0f );
        }

        PRINTF_ASYNC("--- time measure intMollifier()_d dt = 0.0001 --- \n");
        {
            uint32_t start_time = hal::getElapsedMsec();
            
            double c = 0.1;
            for(uint16_t i=1; i<=40; i++){
                c = 0.1 * (double)i;
                double S = intMollifier_d(c, 0.0001);
                
                PRINTF_ASYNC("  %f, %f\n", c, S);
            }
            uint32_t end_time = hal::getElapsedMsec();
            PRINTF_ASYNC("  40 times [ms] |  %d \n", end_time - start_time);
            PRINTF_ASYNC("  ave      [ms] |  %f \n", (float)(end_time - start_time) / 40.0f );
        }
    }

    void TrajectoryInitializer::debugParamSet(){
        PRINTF_ASYNC("--- turnParamSet --- \n");

        for (std::pair<ETurnParamSet, TurnParameter> key_val : _turnParamSet){
            std::string key_str = turnParamSet2Str(key_val.first);
            PRINTF_ASYNC("  --- %s --- \n", key_str.c_str());
            key_val.second.print();
            hal::waitmsec(10);
        }
    }
    
    void TrajectoryInitializer::debugPreCalcs(){
        ETurnType turnList[8] = {ETurnType::TURN_90, ETurnType::TURN_L_90, ETurnType::TURN_180, ETurnType::TURN_S2D_45, ETurnType::TURN_S2D_135, ETurnType::TURN_D2S_45, ETurnType::TURN_D2S_135, ETurnType::TURN_D_90};
        ETurnParamSet paramSetList[8] = {ETurnParamSet::SEARCH, ETurnParamSet::SAFE0, ETurnParamSet::SAFE1, ETurnParamSet::FAST0, ETurnParamSet::FAST1, ETurnParamSet::FAST2, ETurnParamSet::FAST3, ETurnParamSet::FAST4};
        PRINTF_ASYNC("--- turnPreCalcs --- \n");
        for(auto turn : turnList){
            PRINTF_ASYNC("  --- %s --- \n", turnType2Str(turn).c_str());
            PRINTF_ASYNC("    v,     pre,     fol,     end_time\n");
            
            for(auto paramSet : paramSetList){
                float v = getV(paramSet, turn);
                float pre = getPreDist(paramSet, turn);
                float fol = getFolDist(paramSet, turn);
                float end_time = getTrajEndTime(paramSet, turn);
                PRINTF_ASYNC("    %f, %f, %f, %f\n",v, pre, fol, end_time);
            }
            hal::waitmsec(10);
        }
    }


    int usrcmd_trajectoryInitializer(int argc, char **argv){
        
        if (ntlibc_strcmp(argv[1], "debug") == 0 && ntlibc_strcmp(argv[2], "mollifier") == 0) {            
            TrajectoryInitializer::getInstance().debugMollifier();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "debug") == 0 && ntlibc_strcmp(argv[2], "param") == 0) {            
            TrajectoryInitializer::getInstance().debugParamSet();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "debug") == 0 && ntlibc_strcmp(argv[2], "calcs") == 0) {            
            TrajectoryInitializer::getInstance().debugPreCalcs();
            return 0;
        }


        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    }
}
