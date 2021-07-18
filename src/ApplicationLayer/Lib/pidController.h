#pragma once

#include <algorithm>
#include <cmath>

class PidfController {
    public:
    PidfController() {
        Kp = 0.0;
        Ki = 0.0;
        Kd = 0.0;
        F = 0.0;
        reset();
        enable = true;

        integral_saturation_enable = true;
        integral_saturation = -1.0f;
    
        saturation_enable = true;
        saturation = -1.0f;
    }

    virtual void update(float target, float observed_val){
        if(!enable) return;
        
        e_p0 = target - observed_val;
        if(integral_saturation_enable && integral_saturation > 0.0f){
            e_i0 = std::clamp(e_p0 + e_i1, -std::fabs(integral_saturation), std::fabs(integral_saturation));
        }
        else {
            e_i0 = e_p0 + e_i1;
        }

        e_d0 = F * e_d1 + (1.0f - F) * (e_p0 - e_p1);

        if(saturation_enable && saturation > 0.0f){
            u_k0 = std::clamp(Kp * e_p0 + Ki * e_i0 + Kd * e_d0, -std::fabs(saturation), std::fabs(saturation));
        }
        else{
            u_k0 = Kp * e_p0 + Ki * e_i0 + Kd * e_d0;
        }

        e_p1 = e_p0;
        e_i1 = e_i0;
        e_d1 = e_d0;
        u_k1 = u_k0;            
    }

    virtual float getControlVal(){
        if(enable) return u_k0;
        else return 0.0f;
    }

    void set(float Kp_, float Ki_, float Kd_, float F_) {
        Kp = Kp_;
        Ki = Ki_;
        Kd = Kd_;
        F = F_;
    }

    void setIntegralSaturation(float int_saturation) {
        integral_saturation = int_saturation;
        e_i0 = std::clamp(e_i0, -std::fabs(int_saturation), std::fabs(int_saturation));
        e_i1 = std::clamp(e_i0, -std::fabs(int_saturation), std::fabs(int_saturation));
    }

    void setEnable(bool enable_) {
        enable = enable_;
    }

    void setSaturationEnable(bool saturation_enable_) {
        saturation_enable = saturation_enable_;
    }

    void setSaturation(float saturation_){
        saturation = saturation_;
        e_p0 = std::clamp(e_p0, -std::fabs(saturation), std::fabs(saturation));
        e_p1 = std::clamp(e_p1, -std::fabs(saturation), std::fabs(saturation));

        e_i0 = std::clamp(e_i0, -std::fabs(saturation), std::fabs(saturation));
        e_i1 = std::clamp(e_i1, -std::fabs(saturation), std::fabs(saturation));

        e_d0 = std::clamp(e_d0, -std::fabs(saturation), std::fabs(saturation));
        e_d1 = std::clamp(e_d1, -std::fabs(saturation), std::fabs(saturation));

        u_k0 = std::clamp(u_k0, -std::fabs(saturation), std::fabs(saturation));
        u_k1 = std::clamp(u_k1, -std::fabs(saturation), std::fabs(saturation));
    }

    void setIntegralSaturationEnable(bool enable_){
        integral_saturation_enable = enable_;
    }

    float getError(){
        return e_p0;
    }

    virtual void reset() {
        e_p0 = 0.0f;
        e_i0 = 0.0f;
        e_d0 = 0.0f;
        u_k0 = 0.0f;
        e_p1 = 0.0f;
        e_i1 = 0.0f;
        e_d1 = 0.0f;
        u_k1 = 0.0f;
    }

    protected:
    float Kp;
    float Ki;
    float Kd;
    float F;
    float e_p0;
    float e_i0;
    float e_d0;
    float u_k0;
    float e_p1;
    float e_i1;
    float e_d1;
    float u_k1;
    
    bool enable;

    bool integral_saturation_enable;
    float integral_saturation;
    
    bool saturation_enable;
    float saturation;

};


class AngPidfController : public PidfController {
    public:

    virtual void update(float target, float observed_val) {
        if(!enable) return;

        float ang_diff = target - observed_val;
        while(ang_diff >  180.0f) ang_diff -= 360.0f;
        while(ang_diff < -180.0f) ang_diff += 360.0f;
        e_p0 = ang_diff;

        
        if(integral_saturation_enable){
            e_i0 = std::clamp(e_p0 + e_i1, -std::fabs(integral_saturation), std::fabs(integral_saturation));
        }
        else {
            e_i0 = e_p0 + e_i1;
        }

        e_d0 = F * e_d1 + (1.0f - F) * (e_p0 - e_p1);
    
        u_k0 = std::clamp(Kp * e_p0 + Ki * e_i0 + Kd * e_d0, -std::fabs(saturation), std::fabs(saturation));

        e_p1 = e_p0;
        e_i1 = e_i0;
        e_d1 = e_d0;
        u_k1 = u_k0;

    };


};



