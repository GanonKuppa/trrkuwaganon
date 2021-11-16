#include <stdint.h>

#include "iodefine.h"
#include "pd_clock.h"
#include "pd_timer.h"

namespace periferal_driver {

//タイマを三つ使用
//CMTW0 elepsedTimeの下位bit用 (48MHzを8分周でカウント)
//CMTW1 elapsedTimeの上位bit用 (48MHzを512分周でカウント)
//TPU0  割り込み関数内での時間計測用 (48MHzでカウント)

// PCLKBがクロックソース48MHz かつ タイマの分周比=8 --> 1μ秒　＝　6 カウント
    static constexpr uint32_t U_COUNT = 6;
    static constexpr uint32_t M_COUNT = 6000;
    static constexpr uint32_t HRT_U_COUNT = 48;

    void initCMTW0() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(CMTW0) = 0; //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        CMTW0.CMWSTR.BIT.STR = 0; //カウント停止
        CMTW0.CMWCR.BIT.CKS = 0; //0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
        //48MHz,128分周 -> 0.375カウントで1usec   
        //48MHz, 32分周 -> 1.5カウントで1usec    
        //48MHz,  8分周 -> 6カウントで1usec

        CMTW0.CMWCNT = 0;
        CMTW0.CMWCR.BIT.CCLR = 1;
        CMTW0.CMWCR.BIT.CMS = 0; // 32bitカウンタ

        CMTW0.CMWCR.BIT.CCLR = 1; //カウンタのクリア禁止        
    }

    void initCMTW1() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(CMTW1) = 0; //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        CMTW1.CMWCR.BIT.CKS = 3; //0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
        CMTW1.CMWCNT = 0;
        CMTW1.CMWCR.BIT.CCLR = 1;
        CMTW1.CMWCR.BIT.CMS = 0; // 32bitカウンタ

        CMTW1.CMWCR.BIT.CCLR = 1; //カウンタのクリア禁止
        CMTW1.CMWCNT = 0;
    }

    void initTPU0() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA13 = 0; //TPU0-5 ON    //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        TPUA.TSTR.BIT.CST0 = 0;
        TPU0.TCR.BIT.TPSC = 0; //0:PCLK/1 1:PCLK/4 2:PCLK/16
        TPU0.TCNT = 0;
    }


    void startCMTW() {
        CMTW1.CMWSTR.BIT.STR = 1; //カウント開始
        CMTW0.CMWSTR.BIT.STR = 1; //カウント開始        
    }

    void waitClockCount(uint32_t cCount) {        
        uint64_t clock_count_start = (uint64_t)CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
        while (1) {
            uint64_t clock_count_now = (uint64_t)CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
            uint64_t count_diff = clock_count_now - clock_count_start;
            if ( count_diff >= cCount) break;
        }
    }

    void waitnsec(uint32_t nsec) {
        uint64_t clock_count_start = CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
        while (1) {
            uint64_t clock_count_now = CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
            uint64_t elapsed_nsec = (clock_count_now - clock_count_start) / U_COUNT * 1000ull;
            if ( elapsed_nsec >= nsec) break;
        }
    }

    void waitusec(uint32_t usec) {
        uint64_t clock_count_start = CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
        while (1) {
            uint64_t clock_count_now = CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
            uint64_t elapsed_usec = (clock_count_now - clock_count_start) / U_COUNT;
            if ( elapsed_usec >= usec) break;
        }
    }

    void waitmsec(uint32_t msec) {
        uint64_t clock_count_start = (uint64_t)CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
        while (1) {
            uint64_t clock_count_now = (uint64_t)CMTW0.CMWCNT + ((uint64_t)(CMTW1.CMWCNT & 0xfc000000ull) << 6);
            uint64_t elapsed_msec = (clock_count_now - clock_count_start) / M_COUNT;
            if ( elapsed_msec >= msec) break;
        }
    }

    uint64_t getElapsedClockCount(){
        uint32_t cmt1_masked = CMTW1.CMWCNT & 0xfc000000;
        uint64_t clock_count = (uint64_t)CMTW0.CMWCNT | ( (uint64_t)cmt1_masked << 6);
        return clock_count;
    }
    
    uint64_t getElapsedNsec() {
        uint32_t cmt1_masked = CMTW1.CMWCNT & 0xfc000000;
        uint64_t clock_count = (uint64_t)CMTW0.CMWCNT | ( (uint64_t)cmt1_masked << 6);
        return clock_count * 1000 / U_COUNT;
    }

    uint64_t getElapsedUsec() {
        uint32_t cmt1_masked = CMTW1.CMWCNT & 0xfc000000;
        uint64_t clock_count = (uint64_t)CMTW0.CMWCNT | ( (uint64_t)cmt1_masked << 6);
        return clock_count / U_COUNT;
    }

    uint32_t getElapsedMsec() {
        uint32_t cmt1_masked = CMTW1.CMWCNT & 0xfc000000;
        uint64_t clock_count = (uint64_t)CMTW0.CMWCNT | ( (uint64_t)cmt1_masked << 6);
        uint64_t m_sec = clock_count / M_COUNT;
        return (uint32_t)m_sec;
    }

    uint32_t getElapsedSec() {
        return getElapsedMsec() / 1000;
    }

    float calcElapsedUsec(uint64_t clock_count) {
        uint32_t cmt1_masked = CMTW1.CMWCNT & 0xfc000000;
        uint64_t clock_count_now = (uint64_t)CMTW0.CMWCNT | ( (uint64_t)cmt1_masked << 6);
        uint64_t diff = clock_count_now - clock_count;
        return (float)diff / (float)U_COUNT;
    }

    void hrtStartTimer(){
        TPU0.TCNT = 0;
        TPUA.TSTR.BIT.CST0 = 1;
    }

    void hrtStopTimer(){
        TPUA.TSTR.BIT.CST0 = 0; //カウント停止
    }

    float hrtGetElapsedUsec(float usec) {
        float usec_now = TPU0.TCNT / (float)HRT_U_COUNT; 
        return usec_now - usec;
    }
}

