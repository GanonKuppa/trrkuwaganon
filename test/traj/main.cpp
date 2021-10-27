#include <stdio.h>
#include "turnPreCalculation.h"


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

int main() {
    float cp = 100.0f;
    printf("-------- TurnPreCalculation test --------\n");
    
    printf("-- turn_90 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_90, 
            PATH_LENGTH_90,
            TARGET_ANG_90,
            START_ANG_90,
            START_X_90,
            START_Y_90,
            END_X_90,
            END_Y_90,
            v,
            cp,
            DELTA_T
        );
        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_l_90 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_L_90, 
            PATH_LENGTH_L_90,
            TARGET_ANG_L_90,
            START_ANG_L_90,
            START_X_L_90,
            START_Y_L_90,
            END_X_L_90,
            END_Y_L_90,
            v,
            cp,
            DELTA_T
        );

        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_180 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_180, 
            PATH_LENGTH_180,
            TARGET_ANG_180,
            START_ANG_180,
            START_X_180,
            START_Y_180,
            END_X_180,
            END_Y_180,
            v,
            cp,
            DELTA_T
        );

        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_s2d_45 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_S2D_45, 
            PATH_LENGTH_S2D_45,
            TARGET_ANG_S2D_45,
            START_ANG_S2D_45,
            START_X_S2D_45,
            START_Y_S2D_45,
            END_X_S2D_45,
            END_Y_S2D_45,
            v,
            cp,
            DELTA_T
        );

        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_s2d_135 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn_90 = TurnPreCalculation(
            SHAPE_FACTOR_S2D_135, 
            PATH_LENGTH_S2D_135,
            TARGET_ANG_S2D_135,
            START_ANG_S2D_135,
            START_X_S2D_135,
            START_Y_S2D_135,
            END_X_S2D_135,
            END_Y_S2D_135,
            v,
            cp,
            DELTA_T
        );
        float pre_dist = turn_90.getPreDist();
        float fol_dist = turn_90.getFolDist();        
        float beta_max = turn_90.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn_90.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_d_90 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_D_90, 
            PATH_LENGTH_D_90,
            TARGET_ANG_D_90,
            START_ANG_D_90,
            START_X_D_90,
            START_Y_D_90,
            END_X_D_90,
            END_Y_D_90,
            v,
            cp,
            DELTA_T
        );

        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_d2s_45 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_D2S_45, 
            PATH_LENGTH_D2S_45,
            TARGET_ANG_D2S_45,
            START_ANG_D2S_45,
            START_X_D2S_45,
            START_Y_D2S_45,
            END_X_D2S_45,
            END_Y_D2S_45,
            v,
            cp,
            DELTA_T
        );
        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    printf("-- turn_d2s_135 --\n");
    for(float v = 0.1f; v < 1.6f; v +=0.1f){
        auto turn = TurnPreCalculation(
            SHAPE_FACTOR_D2S_135, 
            PATH_LENGTH_D2S_135,
            TARGET_ANG_D2S_135,
            START_ANG_D2S_135,
            START_X_D2S_135,
            START_Y_D2S_135,
            END_X_D2S_135,
            END_Y_D2S_135,
            v,
            cp,
            DELTA_T
        );

        float pre_dist = turn.getPreDist();
        float fol_dist = turn.getFolDist();        
        float beta_max = turn.getBetaAbsMax() * 180.0f/3.14159265f;
        float end_time = turn.getTrajEndTime();
        printf("%5.1f, %10f, %10f, %7.2f, %7.3f \n", v, pre_dist, fol_dist, beta_max, end_time);

    }

    return 0;
}
