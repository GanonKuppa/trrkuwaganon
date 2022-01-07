#include <PeripheralDriverLayer/pd_clock.h>
#include <PeripheralDriverLayer/pd_pwm.h>
#include <stdint.h>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include "iodefine.h"


//49ピン PC3 MOTOR_L_PWM2 MTIOC4B //GTIOC1B-D
//50ピン PC2 MOTOR_L_PWM1 MTIOC4D //GTIOC2B-D
//51ピン PC1 MOTOR_R_PWM1 MTIOC3A
//52ピン PC0 MOTRO_R_PWM2 MTIOC3C

//68ピン PA2 SUC_MOTOR_PWM MTIOC7A
//69ピン PA1 HEATER_PWM MTIOC7B/MTIOC0B/GTIOC2A-C/TIOCB0

namespace peripheral_driver {    
    static float dutyMTU0; //69ピン PA1 HEATER_PWM MTIOC0B
    
    static float dutyMTU3A; //51ピン PC1 MOTOR_R_PWM1 MTIOC3A
    static float dutyMTU3C; //52ピン PC0 MOTRO_R_PWM2 MTIOC3C
    
    static float dutyGPTA1; //49ピン PC3 MOTOR_L_PWM2 GTIOC1B-D
    static float dutyGPTA2; //50ピン PC2 MOTOR_L_PWM1 GTIOC2B-D
    
    static float dutyMTU7; //68ピン PA2 SUC_MOTOR_PWM MTIOC7A

    static constexpr uint16_t FREQ_COUNT = 250;

    void initMTU0() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PA1PFS.BIT.PSEL = 1; //MTIOC0B
        MPC.PWPR.BYTE = 0x80;

        PORTA.PMR.BIT.B1 = 1;

        MTU.TSTRA.BIT.CST0 = 0;
        MTU0.TCR.BIT.TPSC = 0; //PCLKA/1
        MTU0.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア
        

        MTU0.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
        MTU0.TGRA = FREQ_COUNT; //500
        MTU0.TGRB = 1;
        MTU0.TGRC = FREQ_COUNT;
        MTU0.TGRD = 1;
        MTU0.TMDR1.BIT.MD = 3; //PWM2
        MTU0.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
        MTU0.TMDR1.BIT.BFB = 1;
        
        setDutyMTU0(0.0f);
        
    }
    /////////////////////////////////////////////////////////////
    void initMTU3() {
        
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        PORTC.PMR.BIT.B0 = 0;
        PORTC.PMR.BIT.B1 = 0;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PC1PFS.BIT.PSEL = 1; //MTIOC3A
        MPC.PWPR.BYTE = 0x80;        

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PC0PFS.BIT.PSEL = 1; //MTIOC3C
        MPC.PWPR.BYTE = 0x80;        


        MTU.TSTRA.BIT.CST3 = 0;
        MTU3.TCR.BIT.TPSC = 0; //PCLKA/1
        MTU3.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア
        MTU3.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ0出力
        MTU3.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
        MTU3.TIORL.BIT.IOC = 6; //初期出力1 コンペアマッチ0出力
        MTU3.TIORL.BIT.IOD = 5; //初期出力1 コンペアマッチ0出力


        MTU3.TGRA = FREQ_COUNT;
        MTU3.TGRB = 0;
        MTU3.TGRC = FREQ_COUNT;
        MTU3.TGRD = 0;
        MTU3.TMDR1.BIT.MD = 2; //PWMモード1
        
        setDutyMTU3A(0.0f);
        setDutyMTU3C(0.0f);
    }

    /////////////////////////////////////////////////////////////
    void initMTU4() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PC3PFS.BIT.PSEL = 1; //MTIOC4B
        MPC.PC2PFS.BIT.PSEL = 1; //MTIOC4D
        MPC.PWPR.BYTE = 0x80;

        PORT2.PMR.BIT.B1 = 1; //左PWM

        MTU.TSTRA.BIT.CST4 = 0;
        MTU.TOERA.BIT.OE4A = 1; //MTU出力端子を出力許可する

        MTU4.TCR.BIT.TPSC = 0; //PCLKA/1
        MTU4.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        MTU4.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
        MTU4.TIORL.BIT.IOD = 5; //初期出力1 コンペアマッチ0出力

        MTU.TOERA.BIT.OE4B = 1; //MTU出力端子を出力許可する
        MTU.TOERA.BIT.OE4D = 1; //MTU出力端子を出力許可する
        
        MTU4.TGRA = FREQ_COUNT;
        MTU4.TGRB = 0;
        MTU4.TGRC = FREQ_COUNT;
        MTU4.TGRD = 0;
        MTU4.TMDR1.BIT.MD = 2; //PWMモード1
        MTU4.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
        MTU4.TMDR1.BIT.BFB = 1; //バッファーモードに設定
    }

    void initGPTA1(){
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA7 = 0; //GPTAモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        PORTC.PMR.BIT.B3 = 0;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PC3PFS.BIT.PSEL = 0b011110; //GTIOC1B
        MPC.PWPR.BYTE = 0x80;
    
        GPT.GTSTR.BIT.CST1 = 0;
        GPT1.GTIOR.BIT.GTIOB = 0b011001; // 初期出力H 周期の終わりでH出力 コンペアマッチでL
        GPT1.GTONCR.BIT.OBE = 1;
        GPT1.GTPR = FREQ_COUNT;
        GPT1.GTCCRB = 1;
    }

    void initGPTA2(){
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA7 = 0; //GPTAモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        PORTC.PMR.BIT.B2 = 0;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PC2PFS.BIT.PSEL = 0b011110; //GTIOC2B
        MPC.PWPR.BYTE = 0x80;
    
        GPT.GTSTR.BIT.CST2 = 0;
        GPT2.GTIOR.BIT.GTIOB = 0b011001; // 初期出力H 周期の終わりでH出力 コンペアマッチでL
        GPT2.GTONCR.BIT.OBE = 1;
        GPT2.GTPR = FREQ_COUNT;
        GPT2.GTCCRB = 1;

    }



    void initMTU7() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PA2PFS.BIT.PSEL = 0b001000; //MTIOC7A
        MPC.PWPR.BYTE = 0x80;

        PORTA.PMR.BIT.B2 = 1;

        MTU.TSTRB.BIT.CST7 = 0;
        MTU.TOERB.BIT.OE7A = 1; //MTU出力端子を出力許可する

        MTU7.TCR.BIT.TPSC = 0; //PCLKA/1
        MTU7.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        MTU7.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
        MTU7.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
        MTU7.TGRA = FREQ_COUNT;
        MTU7.TGRB = 1;
        MTU7.TGRC = FREQ_COUNT;
        MTU7.TGRD = 1;
        MTU7.TMDR1.BIT.MD = 2; //PWM1
        MTU7.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
        MTU7.TMDR1.BIT.BFB = 1; //バッファーモードに設定
    }



    void setDutyMTU0(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU0 = duty;
        if (fabs(duty) < FLT_EPSILON) {
            PORTA.PMR.BIT.B1 = 0;
            PORTA.PDR.BIT.B1 = 1;
            PORTA.PODR.BIT.B1 = 0;
            //MTU0.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORTA.PMR.BIT.B1 = 0;
            PORTA.PDR.BIT.B1 = 1;
            PORTA.PODR.BIT.B1 = 1;
            //MTU0.TGRD = MTU0.TGRC-1;
        } else {
            PORTA.PMR.BIT.B1= 1;
            MTU0.TGRD = (uint16_t) (MTU0.TGRC * duty);
        }
        MTU.TSTRA.BIT.CST0 = 1;

    }

    void setDutyMTU3A(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU3A = duty;
        if (std::fabs(duty) < FLT_EPSILON) {
            MTU.TSTRA.BIT.CST3 = 0;
            PORTC.PMR.BIT.B1 = 0;
            PORTC.PDR.BIT.B1 = 1;
            PORTC.PODR.BIT.B1 = 0;            
        } else if(duty >= 0.9999f) {
            MTU.TSTRA.BIT.CST3 = 0;
            PORTC.PMR.BIT.B1 = 0;
            PORTC.PDR.BIT.B1 = 1;
            PORTC.PODR.BIT.B1 = 1;            
        } else {
            PORTC.PMR.BIT.B1 = 1;
            MTU3.TGRB = (uint16_t) (MTU3.TGRA * duty);
        }        
        MTU.TSTRA.BIT.CST3 = 1;
    }

    void setDutyMTU3C(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU3C = duty;
        if (std::fabs(duty) < FLT_EPSILON) {
            MTU.TSTRA.BIT.CST3 = 0;
            PORTC.PMR.BIT.B0 = 0;
            PORTC.PDR.BIT.B0 = 1;
            PORTC.PODR.BIT.B0 = 0;
        } else if(duty >= 0.9999f) {
            MTU.TSTRA.BIT.CST3 = 0;
            PORTC.PMR.BIT.B0 = 0;
            PORTC.PDR.BIT.B0 = 1;
            PORTC.PODR.BIT.B0 = 1;
        } else {
            PORTC.PMR.BIT.B0 = 1;
            MTU3.TGRD = (uint16_t) (MTU3.TGRC * duty);
        }        
        MTU.TSTRA.BIT.CST3 = 1;
    }



    void setDutyGPTA1(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyGPTA1 = duty;
        if (std::fabs(duty) < FLT_EPSILON) {
        	GPT.GTSTR.BIT.CST1 = 0;
            PORTC.PMR.BIT.B3 = 0;
            PORTC.PDR.BIT.B3 = 1;
            PORTC.PODR.BIT.B3 = 0;            
        } else if(duty >= 0.9999f) {
        	GPT.GTSTR.BIT.CST1 = 0;
            PORTC.PMR.BIT.B3 = 0;
            PORTC.PDR.BIT.B3 = 1;
            PORTC.PODR.BIT.B3 = 1;            
        } else {
            PORTC.PMR.BIT.B3 = 1;
            GPT1.GTCCRB = (uint16_t) (GPT1.GTPR * duty);
        }        
        GPT.GTSTR.BIT.CST1 = 1;
    }

    void setDutyGPTA2(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyGPTA2 = duty;
        if (std::fabs(duty) < FLT_EPSILON) {
        	GPT.GTSTR.BIT.CST2 = 0;
            PORTC.PMR.BIT.B2 = 0;
            PORTC.PDR.BIT.B2 = 1;
            PORTC.PODR.BIT.B2 = 0;
        } else if(duty >= 0.9999f) {
        	GPT.GTSTR.BIT.CST2 = 0;
            PORTC.PMR.BIT.B2 = 0;
            PORTC.PDR.BIT.B2 = 1;
            PORTC.PODR.BIT.B2 = 1;
        } else {
            PORTC.PMR.BIT.B2 = 1;
            GPT2.GTCCRB = (uint16_t) (GPT2.GTPR * duty);
        }        
        GPT.GTSTR.BIT.CST2 = 1;
    }



    void setDutyMTU7(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU7 = duty;
        if (std::fabs(duty) < FLT_EPSILON) {
            PORTA.PMR.BIT.B2 = 0;
            PORTA.PDR.BIT.B2 = 1;
            PORTA.PODR.BIT.B2 = 0; 
            //MTU4.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORTA.PMR.BIT.B2 = 0; 
            PORTA.PDR.BIT.B2 = 1;
            PORTA.PODR.BIT.B2 = 1; 
            //MTU4.TGRD = MTU4.TGRC-1;
        } else {
            PORTA.PMR.BIT.B2 = 1; 
            MTU7.TGRD = (uint16_t) (MTU7.TGRC * duty);
        }
        MTU.TSTRB.BIT.CST7 = 1;

    }


    float getDutyMTU0() {
        return dutyMTU0;
    }

    float getDutyMTU3A() {
        return dutyMTU3A;
    }

    float getDutyMTU3C() {
        return dutyMTU3C;
    }

    float getDutyGPTA1() {
        return dutyGPTA1;
    }

    float getDutyGPTA2() {
        return dutyGPTA2;
    }

    float getDutyMTU7() {
        return dutyMTU7;
    }

}
