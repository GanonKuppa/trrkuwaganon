#include <stdint.h>

#include "pd_da.h"
#include "iodefine.h"


namespace periferal_driver {
    void initDA() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(DA) = 0;
        SYSTEM.PRCR.WORD = 0xA500;

        PORT0.PDR.BIT.B5 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.P05PFS.BYTE = 0x80;
        MPC.PWPR.BYTE = 0x80;
        PORT0.PMR.BIT.B5 = 0;

        DA.DACR.BYTE = 0x3F;

        DA.DADR1 = 0;
        DA.DACR.BYTE = 0xFF;

    }

    //RX71MのDACは12bit分解能なので0から4095までの値を設定可能
    void setDA(uint16_t da) {
        if(DA.DADR1 != da) {
            DA.DADR1 = da;
            DA.DACR.BIT.DAOE1 = 1;
        }
    }

    uint16_t getDA() {
        return DA.DADR1;
    }
}