#include "trajectoryInitializer.h"

#include <stdint.h>

// Lib
#include "ntlibc.h"
#include "debugLog.h"
#include "mollifier.h"


// Hal
#include "hal_timer.h"

#include ""

namespace module {

    TrajectoryInitializer::TrajectoryInitializer(){
        setModuleName("TrajectoryInitializer");
        _init();
    }

    float getV(ETurnParamSet tp, ETurnType tt){
        return _turnParams.at(tp).getV(tt);
    }

    float getAcc(ETurnParamSet tp, ETurnType tt){
        return _turnParams.at(tp).getAcc(tt);
    }
    
    float getPreDist(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getPreDist(tt);
    }

    float getFolDist(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getFolDist(tt);
    }

    float getTrajEndTime(ETurnParamSet tp, ETurnType tt){
        return _turnPreCalcs.at(tp).getTrajEndTime(tt);
    }

    std::unique_ptr<TurnIterator> generateTurnIterator(ETurnParams tp, ETurnType tt){
        return _turnPreCalcs.at(tp).generateTurnIterator(tt);
    }

    void _init(){
        _turnParams.at(ETurnParamSet::SEARCH) = TurnParameter();
        _turnParams.at(ETurnParamSet::SAFE0) = TurnParameter();
        _turnParams.at(ETurnParamSet::SAFE1) = TurnParameter();
        _turnParams.at(ETurnParamSet::FAST0) = TurnParameter();
        _turnParams.at(ETurnParamSet::FAST1) = TurnParameter();
        _turnParams.at(ETurnParamSet::FAST2) = TurnParameter();
        _turnParams.at(ETurnParamSet::FAST3) = TurnParameter();
        _turnParams.at(ETurnParamSet::FAST4) = TurnParameter();
    }


    void TrajectoryInitializer::debug(){
        
        PRINTF_ASYNC("--- time measure molifire_f() --- \n");
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

        PRINTF_ASYNC("--- time measure molifire_d() --- \n");
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




    int usrcmd_trajectoryInitializer(int argc, char **argv){
        
        if (ntlibc_strcmp(argv[1], "debug") == 0) {
            TrajectoryInitializer::getInstance().debug();
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;        
    }
}
