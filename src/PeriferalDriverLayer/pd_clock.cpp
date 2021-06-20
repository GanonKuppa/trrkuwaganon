#include "iodefine.h"
#include "pd_clock.h"

namespace periferal_driver {

    void initClock() {
        /* ---- Initialization of the non-existent ports ---- */

        /* ---- Initialization of the clock ---- */

        //  SYSTEM.PRCR.WORD = 0xA502;
        //      MSTP(DTC)     = 1;
        //      MSTP(EXDMAC0) = 1;
        //      MSTP(STBYRAM) = 1;
        //      MSTP(ECCRAM)  = 1;
        //      MSTP(RAM0)    = 1;
        //  SYSTEM.PRCR.WORD = 0xA500;
        //R_INIT_Clock();
        SYSTEM.PRCR.WORD = 0xA503;
        //-------HOCOの設定
        SYSTEM.HOCOPCR.BYTE = 0x00;
#define FREQ_16MHz      (0x00)          /* 16 MHz */
#define FREQ_18MHz      (0x01)          /* 18 MHz */
#define FREQ_20MHz      (0x02)          /* 20 MHz */
#define REG_HOCOCR2     (FREQ_16MHz)

        SYSTEM.HOCOCR2.BYTE = REG_HOCOCR2;
        SYSTEM.HOCOCR.BYTE = 0x00;
        while (1 != SYSTEM.OSCOVFSR.BIT.HCOVF) {
            /* Confirm that the Oscillation of the pll clock is stable so the clock is
             available for use as the system clock.*/
        }
        //-------WAITサイクルの設定
#define MEMWAIT_1WAIT       (1)       /* 1wait */
#define MEMWAIT_0WAIT       (0)       /* 0wait */
#define REG_MEMWAIT       (MEMWAIT_0WAIT)
        SYSTEM.MEMWAIT.LONG = REG_MEMWAIT;
        long dummy = SYSTEM.MEMWAIT.LONG * 6;
        dummy = dummy * dummy;
        while (REG_MEMWAIT != SYSTEM.MEMWAIT.BIT.MEMWAIT) {
            /* Wait for writing MEMWAIT */
        }
        for (int i = 0; i < 0x168; i++) {
            ;
        } /* wait over 12ms */
        //-------PLLの設定
        SYSTEM.PLLCR.BIT.PLIDIV = 0b00; // PLL入力分周 1分周
        SYSTEM.PLLCR.BIT.PLLSRCSEL = 0b0; //クロックソース選択 HOCO:0 メインクロック発振器 0
        SYSTEM.PLLCR.BIT.STC = 0b11111; // 0b10111:12逓倍 0b11111:16逓倍
        SYSTEM.PLLCR2.BYTE = 0x00;
        while (1 != SYSTEM.OSCOVFSR.BIT.PLOVF) {
            /* Confirm that the Oscillation of the pll clock is stable so the clock is
             available for use as the system clock.*/
        }

        //-------クロックソースをPLlに設定
        SYSTEM.SCKCR.BIT.ICK = 0b0001; //1: 2分周 0:1分周
        SYSTEM.SCKCR.BIT.PCKA = 0b0001; //2分周
        SYSTEM.SCKCR.BIT.PCKB = 0b0010; //4分周
        SYSTEM.SCKCR.BIT.PCKC = 0b0010; //4分周
        SYSTEM.SCKCR.BIT.PCKD = 0b0010; //4分周
        SYSTEM.SCKCR.BIT.FCK = 0b0010; //4分周
        SYSTEM.SCKCR.BIT.PSTOP0 = 0b1;
        SYSTEM.SCKCR.BIT.PSTOP1 = 0b1;
        SYSTEM.SCKCR.BIT.BCK = 0b0001; //外部バスクロック 2分周
        for (int i = 0; i < 0x168; i++) {
            ;
        } /* wait over 12ms */
        //SYSTEM.SCKCR.LONG = REG_SCKCR;
        //while (REG_SCKCR != SYSTEM.SCKCR.LONG){
        /* Confirm that the written value can be read correctly. */
        //}
        SYSTEM.BCKCR.BYTE = 0x01;
        while (0x01 != SYSTEM.BCKCR.BYTE) {
            /* Confirm that the written value can be read correctly. */
        }
        SYSTEM.SCKCR2.WORD = 0x0041;
        while (0x0041 != SYSTEM.SCKCR2.WORD) {
            /* Confirm that the written value can be read correctly. */
        }
#define CLK_PLL     (0x0400)        /* PLL */
#define CLK_HOCO    (0x0100)        /* HOCO */
#define CLK_SUB     (0x0300)        /* Sub-clock */
#define CLK_MAIN    (0x0200)        /* Main clock */
#define SEL_SYSCLK      (CLK_PLL)
        SYSTEM.SCKCR3.WORD = SEL_SYSCLK;
        SYSTEM.PRCR.WORD = 0xA500;

        //モジュールを再起動する.
        //  SYSTEM.PRCR.WORD = 0xA502;
        //      MSTP(DTC)     = 0;
        //      MSTP(EXDMAC0) = 0;
        //      MSTP(STBYRAM) = 0;
        //      MSTP(ECCRAM)  = 0;
        //      MSTP(RAM0)    = 0;
        //  SYSTEM.PRCR.WORD = 0xA500;

    }
}
