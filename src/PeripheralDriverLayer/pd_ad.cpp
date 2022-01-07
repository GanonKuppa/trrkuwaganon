#include <stdint.h>
#include <iodefine.h>

/*          trr_kuwaganon2  trr_kuwganon1
BATT_AD     PD1 AN109       AN001
SEN_AD_RA   PD0 AN108       AN000
SEN_AD_R    AD2 AN110       AN005
SEN_AD_L    PD3 AN111       AN108
SEN_AD_LA   PE3 AN101       AN101
*/

namespace peripheral_driver {
    void initAD() {
        //SYSTEM.PRCR.WORD = 0xA502;
        //MSTP(S12AD) = 0;
        //SYSTEM.PRCR.WORD = 0xA500;

        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(S12AD1) = 0;
        SYSTEM.PRCR.WORD = 0xA500;

        //AN000
        //PORT4.PDR.BIT.B0 = 0;
        //PORT4.PMR.BIT.B0 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P40PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN001
        //PORT4.PDR.BIT.B1 = 0;
        //PORT4.PMR.BIT.B1 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P41PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN002
        //PORT4.PDR.BIT.B2 = 0;
        //PORT4.PMR.BIT.B2 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P42PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN003
        //PORT4.PDR.BIT.B3 = 0;
        //PORT4.PMR.BIT.B3 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P43PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN004
        //PORT4.PDR.BIT.B4 = 0;
        //PORT4.PMR.BIT.B4 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P44PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN005
        //PORT4.PDR.BIT.B5 = 0;
        //PORT4.PMR.BIT.B5 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P45PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN006
        //PORT4.PDR.BIT.B6 = 0;
        //PORT4.PMR.BIT.B6 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P46PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN007
        //PORT4.PDR.BIT.B7 = 0;
        //PORT4.PMR.BIT.B7 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.P47PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN100
        //PORTE.PDR.BIT.B2 = 0;
        //PORTE.PMR.BIT.B2 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.PE2PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN101 SEN_AD_LA
        PORTE.PDR.BIT.B3 = 0;
        PORTE.PMR.BIT.B3 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PE3PFS.BIT.ASEL = 1;
        MPC.PWPR.BIT.PFSWE = 0;
        MPC.PWPR.BIT.B0WI = 1;

        //AN102
        //PORTE.PDR.BIT.B4 = 0;
        //PORTE.PMR.BIT.B4 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.PE4PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN105
        //PORTE.PDR.BIT.B7 = 0;
        //PORTE.PMR.BIT.B7 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.PE7PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //AN108 SEN_AD_RA
        PORTD.PDR.BIT.B0 = 0;
        PORTD.PMR.BIT.B0 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PD0PFS.BIT.ASEL = 1;
        MPC.PWPR.BIT.PFSWE = 0;
        MPC.PWPR.BIT.B0WI = 1;

        //AN109 BATT_AD
        PORTD.PDR.BIT.B1 = 0;
        PORTD.PMR.BIT.B1 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PD1PFS.BIT.ASEL = 1;
        MPC.PWPR.BIT.PFSWE = 0;
        MPC.PWPR.BIT.B0WI = 1;

        //AN110 SEN_AD_R
        PORTD.PDR.BIT.B2 = 0;
        PORTD.PMR.BIT.B2 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PD2PFS.BIT.ASEL = 1;
        MPC.PWPR.BIT.PFSWE = 0;
        MPC.PWPR.BIT.B0WI = 1;

        //AN111 SEN_AD_L
        PORTD.PDR.BIT.B3 = 0;
        PORTD.PMR.BIT.B3 = 0;
        MPC.PWPR.BIT.B0WI = 0;
        MPC.PWPR.BIT.PFSWE = 1;
        MPC.PD3PFS.BIT.ASEL = 1;
        MPC.PWPR.BIT.PFSWE = 0;
        MPC.PWPR.BIT.B0WI = 1;

        //AN113
        //PORTD.PDR.BIT.B5 = 0;
        //PORTD.PMR.BIT.B5 = 0;
        //MPC.PWPR.BIT.B0WI = 0;
        //MPC.PWPR.BIT.PFSWE = 1;
        //MPC.PD5PFS.BIT.ASEL = 1;
        //MPC.PWPR.BIT.PFSWE = 0;
        //MPC.PWPR.BIT.B0WI = 1;

        //S12AD.ADCSR.BIT.EXTRG = 1;
        //S12AD.ADCSR.BIT.TRGE = 1;
        //S12AD.ADCSR.BIT.ADCS = 0;
        //S12AD.ADADC.BIT.ADC  = 0; //3:加算回数の選択 4times変換 3times加算
        //S12AD.ADADC.BIT.AVEE = 1; //1:加算後の値は平均        
        //S12AD.ADCER.BIT.ADRFMT = 0; //右詰めフォーマット
        //S12AD.ADADS0.BIT.ADS0 = 0x0000; //1:変換値加算モード選択 0：非選択

        S12AD1.ADCSR.BIT.EXTRG = 1;
        S12AD1.ADCSR.BIT.TRGE = 1;
        S12AD1.ADCSR.BIT.ADCS = 0;
        //S12AD1.ADADC.BIT.ADC  = 0; //3:加算回数の選択 4times変換 3times加算
        //S12AD1.ADADC.BIT.AVEE = 1; //1:加算後の値は平均
        S12AD1.ADCER.BIT.ADRFMT = 0; //右詰めフォーマット
        //S12AD1.ADADS0.BIT.ADS0 = 0x0000; //1:変換値加算モード選択 0：非選択
        //S12AD1.ADADS1.BIT.ADS1 = 0x0000; //1:変換値加算モード選択 0：非選択
        
    }

    uint16_t startAD_AN000() {
        S12AD.ADANSA0.WORD = 0x0001;
                
        S12AD.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR0;
    }

    uint16_t startAD_AN001() {
        S12AD.ADANSA0.WORD = 0x0002;
              
        S12AD.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR1;
    }

    uint16_t startAD_AN002() {
        S12AD.ADANSA0.WORD = 0x0004;

        S12AD.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR2;
    }

    uint16_t startAD_AN003() {
        S12AD.ADANSA0.WORD = 0x0008;
        
        S12AD.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR3;
    }

    uint16_t startAD_AN004() {
        S12AD.ADANSA0.WORD = 0x0010;
        
        S12AD.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1)dummy ++;
        return S12AD.ADDR4;
    }

    uint16_t startAD_AN005() {
        S12AD.ADANSA0.WORD = 0x0020;
        
        S12AD.ADCSR.BIT.ADST = 1;        
        int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR5;
    }

    uint16_t startAD_AN006() {
        S12AD.ADANSA0.WORD = 0x0040;
        
        S12AD.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR6;
    }

    uint16_t startAD_AN007() {
        S12AD.ADANSA0.WORD = 0x0080;
        
        S12AD.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD.ADDR7;
    }

    uint16_t startAD_AN100() {
        S12AD1.ADANSA0.WORD = 0x0001;
        
        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR0;
    }

    uint16_t startAD_AN101() {
        S12AD1.ADANSA0.WORD = 0x0002;
        
        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR1;
    }

    uint16_t startAD_AN102() {
        S12AD1.ADANSA0.WORD = 0x0004;

        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR2;
    }

    uint16_t startAD_AN103() {
        S12AD1.ADANSA0.WORD = 0x0008;

        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1)dummy ++;
        return S12AD1.ADDR3;
    }

    uint16_t startAD_AN104() {
        S12AD1.ADANSA0.WORD = 0x0010;

        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR4;
    }

    uint16_t startAD_AN105() {
        S12AD1.ADANSA0.WORD = 0x0020;

        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR5;
    }

    uint16_t startAD_AN106() {
        S12AD1.ADANSA0.WORD = 0x0040;

        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR6;
    }

    uint16_t startAD_AN107() {
        S12AD1.ADANSA0.WORD = 0x0040;
        
        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR7;
    }

    uint16_t startAD_AN108() {
        S12AD1.ADANSA0.WORD = 0x0100;
        
        S12AD1.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR8;
    }

    uint16_t startAD_AN109() {
        S12AD1.ADANSA0.WORD = 0x0200;
        
        S12AD1.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR9;
    }

    uint16_t startAD_AN110() {
        S12AD1.ADANSA0.WORD = 0x0400;
        
        S12AD1.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1)  dummy ++;            
        return S12AD1.ADDR10;
    }

    uint16_t startAD_AN111() {
        S12AD1.ADANSA0.WORD = 0x0800;

        S12AD1.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR11;
    }

    uint16_t startAD_AN112() {
        S12AD1.ADANSA0.WORD = 0x1000;

        S12AD1.ADCSR.BIT.ADST = 1;
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR12;
    }

    uint16_t startAD_AN113() {
        S12AD1.ADANSA0.WORD = 0x2000;

        S12AD1.ADCSR.BIT.ADST = 1;        
        volatile int dummy = 0;
        while (S12AD1.ADCSR.BIT.ADST == 1) dummy ++;
        return S12AD1.ADDR13;
    }


    uint16_t getAD_AN000() {
        return S12AD.ADDR0;
    }
    uint16_t getAD_AN001() {
        return S12AD.ADDR1;
    }
    uint16_t getAD_AN002() {
        return S12AD.ADDR2;
    }
    uint16_t getAD_AN003() {
        return S12AD.ADDR3;
    }
    uint16_t getAD_AN004() {
        return S12AD.ADDR4;
    }
    uint16_t getAD_AN005() {
        return S12AD.ADDR5;
    }
    uint16_t getAD_AN006() {
        return S12AD.ADDR6;
    }
    uint16_t getAD_AN007() {
        return S12AD.ADDR7;
    }

    uint16_t getAD_AN100() {
        return S12AD1.ADDR0;
    }
    uint16_t getAD_AN101() {
        return S12AD1.ADDR1;
    }
    uint16_t getAD_AN102() {
        return S12AD1.ADDR2;
    }
    uint16_t getAD_AN103() {
        return S12AD1.ADDR3;
    }
    uint16_t getAD_AN104() {
        return S12AD1.ADDR4;
    }
    uint16_t getAD_AN105() {
        return S12AD1.ADDR5;
    }
    uint16_t getAD_AN106() {
        return S12AD1.ADDR6;
    }
    uint16_t getAD_AN107() {
        return S12AD1.ADDR7;
    }

    uint16_t getAD_AN108() {
        return S12AD1.ADDR8;
    }
    uint16_t getAD_AN109() {
        return S12AD1.ADDR9;
    }
    uint16_t getAD_AN110() {
        return S12AD1.ADDR10;
    }

    uint16_t getAD_AN111() {
        return S12AD1.ADDR11;
    }
    uint16_t getAD_AN112() {
        return S12AD1.ADDR12;
    }
    uint16_t getAD_AN113() {
        return S12AD1.ADDR13;
    }


}
