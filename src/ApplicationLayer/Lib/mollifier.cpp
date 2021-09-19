#include "mollifier.h"

#include <cmath>

// Lib
#include "intMollifierTbl.h"
#include "tableLinearInterpolation.h"


double mollifier_d(double x, double a, double b, double c){
    if(c <= 0.0 || x <= 0.0 || x >= b){
        return 0.0;
    }        

    double e = 2.71828182845904523536;
    double val = std::pow(std::abs(2.0 * x/b - 1.0), c);
    return e / a * exp( (-1.0)/(1.0 - val));
}

double intMollifier_d(double c, double dt){
    double S = 0.0;
    double t = 0.0;

    while(t < 1.0f){
        S += mollifier_d(t, 1.0, 1.0, c) * dt;
        t += dt;
    }
    return S;        
}

float mollifier_f(float x, float a, float b, float c){
    if(c <= 0.0f || x <= 0.0f || x >= b){
        return 0.0f;
    }        

    float e = 2.71828182845904523536f;
    float val = std::pow(std::fabs(2.0f * x/b - 1.0f), c);
    return e / a * expf( (-1.0f)/(1.0f - val));
}

float intMollifier_f(float c, float dt){
    float S = 0.0f;
    float t = 0.0f;

    while(t < 1.0f){
        S += mollifier_f(t, 1.0f, 1.0f, c) * dt;
        t += dt;
    }
    return S;
}

float  intMollifier_f_tbl(float c){
    return tableLinearInterpolation(c, INT_MOLLIFIER_F_IN_MIN, INT_MOLLIFIER_F_IN_MAX, INT_MOLLIFIER_F_IN_DELTA, INT_MOLLIFIER_F_TBL, INT_MOLLIFIER_F_TBL_SIZE);
};

