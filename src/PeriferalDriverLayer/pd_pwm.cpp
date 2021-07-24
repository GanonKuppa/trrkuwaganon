#include <stdint.h>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include "iodefine.h"

#include "pd_clock.h"
#include "pd_pwm.h"

//49ピン PC3 MOTOR_L_PWM2 MTIOC4B
//50ピン PC2 MOTOR_L_PWM1 MTIOC4D
//51ピン PC1 MOTOR_R_PWM1 MTIOC3A
//52ピン PC0 MOTRO_R_PWM2 MTIOC3C

//68ピン PA2 SUC_MOTOR_PWM MTIOC7A
//69ピン PA1 HEATER_PWM MTIOC7B/MTIOC0B/GTIOC2A-C/TIOCB0

namespace periferal_driver {    
    static float dutyMTU0; //P13
    static float dutyMTU3; //P14
    static float dutyMTU4; //P21
    static float dutyTPU3; //P20
    static float dutyMTU7; //PA2

    static constexpr uint16_t FREQ_COUNT = 400;

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

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.P14PFS.BIT.PSEL = 1; //MTIOC3A
        MPC.PWPR.BYTE = 0x80;

        PORT1.PMR.BIT.B4 = 1;

        MTU.TSTRA.BIT.CST3 = 0;
        MTU3.TCR.BIT.TPSC = 0; //PCLKA/1
        MTU3.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア
        MTU3.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
        MTU3.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
        MTU3.TGRA = FREQ_COUNT; //500
        MTU3.TGRB = 1;
        MTU3.TGRC = FREQ_COUNT;
        MTU3.TGRD = 1;
        MTU3.TMDR1.BIT.MD = 2; //PWM1
        MTU3.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
        MTU3.TMDR1.BIT.BFB = 1;
    }

    /////////////////////////////////////////////////////////////
    void initMTU4() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA9 = 0; //MTUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.P21PFS.BIT.PSEL = 0b001000; //MTIOC4A
        MPC.PWPR.BYTE = 0x80;

        PORT2.PMR.BIT.B1 = 1; //左PWM

        MTU.TSTRA.BIT.CST4 = 0;
        MTU.TOERA.BIT.OE4A = 1; //MTU出力端子を出力許可する

        MTU4.TCR.BIT.TPSC = 0; //PCLKA/1
        MTU4.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        MTU4.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
        MTU4.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力
        MTU4.TGRA = FREQ_COUNT;
        MTU4.TGRB = 1;
        MTU4.TGRC = FREQ_COUNT;
        MTU4.TGRD = 1;
        MTU4.TMDR1.BIT.MD = 2; //PWM1
        MTU4.TMDR1.BIT.BFA = 1;   //バッファーモードに設定
        MTU4.TMDR1.BIT.BFB = 1; //バッファーモードに設定
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



    void initTPU3() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA13 = 0; //TPUモジュールON
        SYSTEM.PRCR.WORD = 0xA500;

        PORT2.PMR.BIT.B0 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.P20PFS.BIT.PSEL = 3;
        MPC.PWPR.BYTE = 0x80;

        PORT2.PMR.BIT.B0 = 1;

        TPUA.TSTR.BIT.CST3 = 0;

        TPU3.TCR.BIT.TPSC = 0; //PCLKA/1
        TPU3.TCR.BIT.CCLR = 1; //PWM TGRAのコンペアマッチでTCNTクリア TGRDは6
        TPU3.TIORH.BIT.IOA = 6; //初期出力1 コンペアマッチ1出力
        TPU3.TIORH.BIT.IOB = 5; //初期出力1 コンペアマッチ0出力

        TPU3.TGRA = FREQ_COUNT; //500
        TPU3.TGRB = 1;
        TPU3.TGRC = FREQ_COUNT;
        TPU3.TGRD = 1;
        TPU3.TMDR.BIT.MD = 3; //PWM2
        TPU3.TMDR.BIT.BFA = 1;
        TPU3.TMDR.BIT.BFB = 1;
    }


    void setDutyMTU0(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU0 = duty;
        if (fabs(duty) < FLT_EPSILON) {
            PORTA.PMR.BIT.B1 = 0;
            PORTA.PODR.BIT.B1 = 0;
            //MTU0.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORTA.PMR.BIT.B1 = 0;
            PORTA.PODR.BIT.B1 = 1;
            //MTU0.TGRD = MTU0.TGRC-1;
        } else {
            PORTA.PMR.BIT.B1= 1;
            MTU0.TGRD = (uint16_t) (MTU0.TGRC * duty);
        }
        MTU.TSTRA.BIT.CST0 = 1;

    }


    void setDutyMTU3(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU3 = duty;
        if (fabs(duty) < FLT_EPSILON) {
            PORT1.PMR.BIT.B4 = 0;
            PORT1.PODR.BIT.B4 = 0;
            //MTU3.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORT1.PMR.BIT.B4 = 0;
            PORT1.PODR.BIT.B4 = 1;
            //MTU3.TGRD = MTU3.TGRC -1;
        } else {
            PORT1.PMR.BIT.B4 = 1;
            MTU3.TGRD = (uint16_t) (MTU3.TGRC * duty);
        }
        MTU.TSTRA.BIT.CST3 = 1;

    }

    void setDutyMTU4(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU4 = duty;
        if (fabs(duty) < FLT_EPSILON) {
            PORT2.PMR.BIT.B1 = 0; //左PWM
            PORT2.PODR.BIT.B1 = 0; //左PWM
            //MTU4.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORT2.PMR.BIT.B1 = 0; //左PWM
            PORT2.PODR.BIT.B1 = 1; //左PWM
            //MTU4.TGRD = MTU4.TGRC-1;
        } else {
            PORT2.PMR.BIT.B1 = 1; //左PWM
            MTU4.TGRD = (uint16_t) (MTU4.TGRC * duty);
        }
        MTU.TSTRA.BIT.CST4 = 1;

    }

    void setDutyMTU7(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU7 = duty;
        if (fabs(duty) < FLT_EPSILON) {
            PORTA.PMR.BIT.B2 = 0; 
            PORTA.PODR.BIT.B2 = 0; 
            //MTU4.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORTA.PMR.BIT.B2 = 0; 
            PORTA.PODR.BIT.B2 = 1; 
            //MTU4.TGRD = MTU4.TGRC-1;
        } else {
            PORTA.PMR.BIT.B2 = 1; 
            MTU7.TGRD = (uint16_t) (MTU7.TGRC * duty);
        }
        MTU.TSTRB.BIT.CST7 = 1;

    }


    void setDutyTPU3(float duty) {
        duty = std::clamp<float>(duty, 0.0f, 1.0f);
        dutyMTU4 = duty;
        if (fabs(duty) < FLT_EPSILON) {
            PORT2.PMR.BIT.B0 = 0;
            PORT2.PODR.BIT.B0 = 0;

            //TPU3.TGRD = 1;
        } else if(duty >= 1.0f) {
            PORT2.PMR.BIT.B0 = 0;
            PORT2.PODR.BIT.B0 = 1;
            //TPU3.TGRD = TPU3.TGRC -1;
        } else {
            PORT2.PMR.BIT.B0 = 1;
            TPU3.TGRD = (uint16_t) (TPU3.TGRC * duty);
        }
        TPUA.TSTR.BIT.CST3 = 1;

    }



    float getDutyMTU0() {
        return dutyMTU0;
    }

    float getDutyMTU3() {
        return dutyMTU3;
    }

    float getDutyMTU4() {
        return dutyMTU4;
    }

    float getDutyTPU3() {
        return dutyTPU3;
    }

    float getDutyMTU7() {
        return dutyMTU7;
    }

}
