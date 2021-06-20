#include <stdint.h>
#include "iodefine.h"
#include "pd_phaseCounting.h"


namespace periferal_driver {
    void initMTU1() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(MTU1) = 0;
        SYSTEM.PRCR.WORD = 0xA500;

        MTU.TSTRA.BIT.CST1 = 0;
        MTU1.TCNT = 0;

        PORT2.PDR.BIT.B5 = 0;
        PORT2.PMR.BIT.B5 = 0;
        PORT2.PDR.BIT.B4 = 0;
        PORT2.PMR.BIT.B4 = 0;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.P25PFS.BYTE = 2; //MTCLKB
        MPC.P24PFS.BYTE = 2; //MTCLKA
        MPC.PWPR.BYTE = 0x80;

        PORT2.PMR.BIT.B5 = 1;
        PORT2.PMR.BIT.B4 = 1;

        MTU1.TMDR1.BIT.MD = 4; //位相計数モード1

        MTU.TSTRA.BIT.CST1 = 1;

    }

    void initMTU2() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(MTU2) = 0;
        SYSTEM.PRCR.WORD = 0xA500;

        MTU.TSTRA.BIT.CST2 = 0;
        MTU2.TCNT = 0;

        PORT2.PDR.BIT.B3 = 0;
        PORT2.PMR.BIT.B3 = 0;
        PORT2.PDR.BIT.B2 = 0;
        PORT2.PMR.BIT.B2 = 0;

        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.P23PFS.BYTE = 2; //MTCLKC
        MPC.P22PFS.BYTE = 2; //MTCLKD
        MPC.PWPR.BYTE = 0x80;

        PORT2.PMR.BIT.B3 = 1;
        PORT2.PMR.BIT.B2 = 1;

        MTU2.TMDR1.BIT.MD = 4; //位相計数モード1

        MTU.TSTRA.BIT.CST2 = 1;

    }

    uint16_t getCountMTU1() {
        return MTU1.TCNT;
    }

    uint16_t getCountMTU2() {
        return MTU2.TCNT;
    }
}
