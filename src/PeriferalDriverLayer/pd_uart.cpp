#include <stdint.h>
#include <deque>

#include "pd_uart.h"
#include "iodefine.h"

using std::deque;

namespace periferal_driver {

    void initSCI1() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP( SCI1 ) = 0; //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        PORT3.PMR.BIT.B0 = 0; //ピンの設定をするときはまずピンを汎用ポートに設定しておく
        PORT2.PMR.BIT.B6 = 0;
        MPC.PWPR.BIT.B0WI = 0; //書き込み許可 0で許可
        MPC.PWPR.BIT.PFSWE = 1; //書き込み許可  1で許可
        MPC.P26PFS.BIT.PSEL = 10; //P26を TXD1に
        MPC.P30PFS.BIT.PSEL = 10; //P30を RXD1に
        MPC.PWPR.BIT.PFSWE = 0; //PFSWEの書き込み禁止 0
        MPC.PWPR.BIT.B0WI = 1; //書き込み禁止
        PORT3.PMR.BIT.B0 = 1; //周辺モジュールをピンに割り当て
        PORT2.PMR.BIT.B6 = 1; //

        SCI1.SMR.BIT.CKS = 0; //PCLK/1
        SCI1.SMR.BIT.MP = 0;
        SCI1.SMR.BIT.STOP = 0;
        SCI1.SMR.BIT.PM = 0;
        SCI1.SMR.BIT.PE = 0;
        SCI1.SMR.BIT.CHR = 0;
        SCI1.SMR.BIT.CM = 0;
        SCI1.SEMR.BIT.ABCS = 1;
        SCI1.SEMR.BIT.BGDM = 1;
        SCI1.BRR = 3 - 1; // N = PCLK /(8*BRR)

        SCI1.SCR.BIT.RE = 1; //受信許可
        SCI1.SCR.BIT.TE = 1; //送信許可

    }

    void put1byteSCI1(uint8_t c) {
        while ( SCI1.SSR.BIT.TEND == 0);
        SCI1.TDR = c;
    }

    void putnbyteSCI1(uint8_t* buf, uint16_t len) {
        for (uint16_t i = 0; i < len; i++) {
            put1byteSCI1(buf[i]);
        }
    }

    void initSCIFA9() {
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP( SCIFA9 ) = 0; //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        PORTB.PMR.BIT.B6 = 0; //ピンの設定をするときはまずピンを汎用ポートに設定しておく
        PORTB.PMR.BIT.B7 = 0;
        MPC.PWPR.BIT.B0WI = 0; //書き込み許可 0で許可
        MPC.PWPR.BIT.PFSWE = 1; //書き込み許可  1で許可
        MPC.PB6PFS.BIT.PSEL = 0b1010; //RXD9に
        MPC.PB7PFS.BIT.PSEL = 0b1010; //TXD2に
        MPC.PWPR.BIT.PFSWE = 0; //PFSWEの書き込み禁止 0
        MPC.PWPR.BIT.B0WI = 1; //書き込み禁止
        PORTB.PMR.BIT.B6 = 1; //周辺モジュールをピンに割り当て
        PORTB.PMR.BIT.B7 = 1; //

        SCIFA9.SMR.BIT.CKS = 0;
        SCIFA9.SEMR.BIT.ABCS0 = 1;
        SCIFA9.SEMR.BIT.BGDM = 1;
        SCIFA9.BRR = 6 - 1; //6-1:2Mbps    13-1:921600bps
        SCIFA9.FCR.BIT.TTRG = 0b11;

        //受信設定
        SCIFA9.FCR.BIT.RTRG = 0b11;
        SCIFA9.SCR.BIT.RE = 1;

        //送信設定
        //IEN(SCIFA9,TXIF9) = 1;//割り込み要求を許可
        //IR(SCIFA9,TXIF9)=0;//割り込みステータフラグをクリア

        //SCIFA9.SCR.BIT.TEIE = 1;
        SCIFA9.SCR.BIT.TE = 1;
        //SCIFA9.SCR.BIT.TIE = 1;

    }

    /***********SCIFA9用送受信バッファ*******************/
    static std::deque<uint8_t> sendBuf; //送信用データバッファ
    static std::deque<uint8_t> recvBuf;//受信用データバッファ

    std::deque<uint8_t>& getSendBufSCIFA9() {
        return sendBuf;
    }

    std::deque<uint8_t>& getRecvBufSCIFA9() {
        return recvBuf;
    }

    /***********nbyte送信関数*******************/
    void putnbyteSCIFA9(uint8_t* buf, uint16_t len) {
        for (uint16_t i = 0; i < len; i++) {
            sendBuf.push_back(buf[i]);
        }
    }
    /***********受信バッファの中身を取り出す関数******************/
    //この関数はタイマ割り込み関数内で周期的に呼び出すこと
    void recvDataSCIFA9() {
        uint8_t count = 0;
        if(SCIFA9.FSR.BIT.BRK == 1)SCIFA9.FSR.BIT.BRK = 0;
        if(SCIFA9.LSR.BIT.ORER == 1)SCIFA9.LSR.BIT.ORER = 0;
        if(SCIFA9.FDR.BIT.R == 0) return;
        while (SCIFA9.FDR.BIT.R != 0 && count < 16) {
            recvBuf.push_back((uint8_t)(SCIFA9.FRDR));
            count++;
        }
    }


    /*************送信バッファの中身を送信する関数***************/
    //この関数はタイマ割り込み関数内で周期的に呼び出すこと
    void sendDataSCIFA9() {
        //if (SCIFA9.FDR.BIT.T == 0x) return;
        uint8_t count = 0;
        while (SCIFA9.FDR.BIT.T < 0x10 && count < 16) {
            if (sendBuf.empty() == true) return;
            SCIFA9.FTDR = sendBuf.front();
            sendBuf.pop_front();
            count++;
        }
    }

}
