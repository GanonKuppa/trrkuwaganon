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
    static bool send_lock = false;
    static bool recv_lock = false;
    static const int SEND_BUF_MAX = 2048;
    static const int RECV_BUF_MAX = 512;

    /***********nbyte送信関数*******************/
    void putnbyteSCIFA9(uint8_t* buf, uint16_t len) {
        if(send_lock) return;
        send_lock = true;
        for (uint16_t i = 0; i < len; i++) {
            if(sendBuf.size() < SEND_BUF_MAX){
                sendBuf.push_back(buf[i]);
            }
        }
        send_lock = false;
    }

    bool readnbyteSCIFA9(uint8_t* buf, uint16_t len) {
        if(recv_lock) return true;
        
        recv_lock = true;
        if(recvBuf.size() < len){
            recv_lock = false;
            return false;
        }
        else{
            recv_lock = true;
            for(int i=0;i<len;i++){
                buf[i] = recvBuf.front();
                recvBuf.pop_front();
            }
            recv_lock = false;
            return true;
        }

    }

    bool isEmptyRecvBufSCIFA9() {
        if(recv_lock) return true;
        recv_lock = true;
        bool empty = recvBuf.empty();
        recv_lock = false;
        return empty;
    }

    uint16_t getSCIFA9Bufsize(){
        if(recv_lock) return 0;
        
        recv_lock = true;
        uint16_t size = recvBuf.size();
        recv_lock = false;
        return size;
    }


    /***********受信バッファの中身を取り出す関数******************/
    //この関数はタイマ割り込み関数内で周期的に呼び出すこと
    void recvDataSCIFA9() {
        if(recv_lock) return;

        uint8_t count = 0;
        if(SCIFA9.FSR.BIT.BRK == 1)SCIFA9.FSR.BIT.BRK = 0;
        if(SCIFA9.LSR.BIT.ORER == 1)SCIFA9.LSR.BIT.ORER = 0;
        if(SCIFA9.FDR.BIT.R == 0) return;
        
        recv_lock = true;
        while (SCIFA9.FDR.BIT.R != 0 && count < 16) {
            if(recvBuf.size() < RECV_BUF_MAX){
                recvBuf.push_back((uint8_t)(SCIFA9.FRDR));
            }
            count++;
        }
        recv_lock = false;
    }


    /*************送信バッファの中身を送信する関数***************/
    //この関数はタイマ割り込み関数内で周期的に呼び出すこと
    void sendDataSCIFA9() {
        //if (SCIFA9.FDR.BIT.T == 0x) return;
        if(send_lock) return;

        send_lock = true;
        uint8_t count = 0;
        while (SCIFA9.FDR.BIT.T < 0x10 && count < 16) {
            if (sendBuf.empty() == true){
                send_lock = false;
                return;
            }
            SCIFA9.FTDR = sendBuf.front();
            sendBuf.pop_front();
            count++;
        }
        send_lock = false;
    }

}
