#include <stdint.h>
#include "pd_timerInterrupt.h"
#include "iodefine.h"

namespace periferal_driver {
    volatile static uint32_t uCountIntCMT0 = 0;
    volatile static uint32_t uCountIntCMT1 = 0;

    //タイマ割り込みを設定

    void initCMT0() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //CMT0と1 ON
        SYSTEM.PRCR.WORD = 0xA500;

        CMT0.CMCR.BIT.CMIE = 1; //割り込みを許可
        IEN(CMT0,CMI0)= 1; //割り込み要求を許可

        CMT0.CMCR.BIT.CKS = 0; //0:8分周 1:32分周 2:128分周 3:512分周
        CMT0.CMCOR = 1500 - 1; // 1500:250usec 6000:1msec

        IR(CMT0,CMI0)=0; //割り込みステータフラグをクリア

    }

    void setPriorityCMT0(uint8_t priori) {
        IPR(CMT0,CMI0)= priori; //割り込み優先度 15が最高
    }

    void startCMT0() {
        CMT.CMSTR0.BIT.STR0 = 1;
    }

    void stopCMT0() {
        CMT.CMSTR0.BIT.STR0 = 0;
    }

    uint32_t endTimeuCountIntCMT0() {
        uCountIntCMT0 = CMT0.CMCNT / 6;
        return uCountIntCMT0;
    }

    uint32_t getTimeuCountIntCMT0() {
        return uCountIntCMT0;
    }

    void initCMT1() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; //CMT0と1 ON
        SYSTEM.PRCR.WORD = 0xA500;

        CMT1.CMCR.BIT.CMIE = 1; //割り込みを許可
        IEN(CMT1,CMI1)= 1; //割り込み要求を許可

        CMT1.CMCR.BIT.CKS = 0; //0:8分周 1:32分周 2:128分周 3:512分周
        CMT1.CMCOR = 60 - 1; //60:10usec 300:50usec 1500:250usec 6000:1msec

        IR(CMT0,CMI0)=0; //割り込みステータフラグをクリア

    }

    void setPriorityCMT1(uint8_t priori) {
        IPR(CMT1,CMI1)= priori; //割り込み優先度 15が最高
    }

    void startCMT1() {
        CMT.CMSTR0.BIT.STR1 = 1;
    }

    void stopCMT1() {
        CMT.CMSTR0.BIT.STR1 = 0;
    }

    uint32_t endTimeuCountIntCMT1() {
        uCountIntCMT1 = CMT1.CMCNT / 6;
        return uCountIntCMT1;
    }

    uint32_t getTimeuCountIntCMT1() {
        return uCountIntCMT1;
    }


}
