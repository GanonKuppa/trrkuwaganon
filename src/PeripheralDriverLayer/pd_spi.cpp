#include <PeripheralDriverLayer/pd_spi.h>
#include <stdint.h>
#include "iodefine.h"



namespace peripheral_driver {

    void initRSPI0() {
        int dummy;
        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(RSPI0) = 0; //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        //各ピンをSPIに割り付け
        PORTA.PMR.BIT.B4 = 0; //ピンの設定をするときはまずピンを汎用ポートに設定しておく
        PORTA.PMR.BIT.B5 = 0;
        PORTA.PMR.BIT.B6 = 0;
        PORTA.PMR.BIT.B7 = 0;
        
        MPC.PWPR.BIT.B0WI = 0; //書き込み許可 0で許可
        MPC.PWPR.BIT.PFSWE = 1; //書き込み許可  1で許可
        //MPC.PA4PFS.BIT.PSEL = 0b001101; //SSLA0に設定 

        MPC.PA5PFS.BIT.PSEL = 0b001101; //RSPCKAに設定
        MPC.PA6PFS.BIT.PSEL = 0b001101; //MOSIAに設定
        MPC.PA7PFS.BIT.PSEL = 0b001101; //MISOAに設定

        PORTA.PDR.BIT.B4 = 1;  //SSLA0はGPIOとしてCSを制御
        PORTA.PODR.BIT.B4 = 1;


        MPC.PWPR.BIT.PFSWE = 0; //PFSWEの書き込み禁止 0
        MPC.PWPR.BIT.B0WI = 1; //書き込み禁止

        //PORTA.PMR.BIT.B4 = 1; //ピンの設定をするときはまずピンを汎用ポートに設定しておく
        PORTA.PMR.BIT.B5 = 1;
        PORTA.PMR.BIT.B6 = 1;
        PORTA.PMR.BIT.B7 = 1;
        

        RSPI0.SPDCR.BIT.SPLW = 0; //SPDRレジスタへはワードアクセス

        //RSPIビットレートレジスタ(SPBR): 通常のシリアル通信と同様に設定する
        //ビットレート = f(PCLKA)/[2 × (n + 1) × 2^N]
        //n=SPBR, N=BRDV
        //mpu9250は1MHzまでOK
        //icm20608gは8MHzまでOK
        //asm330lhhは10MHzまでOK
        RSPI0.SPBR = 4; //9.6MHz        

        //RSPI制御レジスタ(SPCR)
        RSPI0.SPCR.BIT.SPMS = 0;
        RSPI0.SPCR.BIT.TXMD = 0;

        //RSPIコマンドレジスタ(SPCMDx): 送受信フォーマットの設定
        RSPI0.SPCMD0.BIT.BRDV = 0;
        RSPI0.SPCMD0.BIT.CPHA = 1;
        RSPI0.SPCMD0.BIT.CPOL = 1;
        RSPI0.SPCMD0.BIT.SSLA = 0; //0:SSL0 1:SSL1
        RSPI0.SPCMD0.BIT.SSLKP = 1;
        RSPI0.SPDCR.BIT.SPFC = 0b00; //バッファのフレーム数を指定

        RSPI0.SPDCR.BIT.SPRDTD = 0; //受信バッファを読み出す
        RSPI0.SPCR.BIT.MSTR = 1;
        dummy = RSPI0.SPCR.BIT.MSTR;
    }

    void initRSPI1() {
        int dummy;

        SYSTEM.PRCR.WORD = 0xA502;
        MSTP(RSPI1) = 0; //モジュールストップを解除
        SYSTEM.PRCR.WORD = 0xA500;

        //各ピンをSPIに割り付け
        PORTE.PMR.BIT.B4 = 0; //ピンの設定をするときはまずピンを汎用ポートに設定しておく
        PORTE.PMR.BIT.B5 = 0;
        PORTE.PMR.BIT.B6 = 0;
        PORTE.PMR.BIT.B7 = 0;
        PORTA.PMR.BIT.B0 = 0;

        MPC.PWPR.BIT.B0WI = 0; //書き込み許可 0で許可
        MPC.PWPR.BIT.PFSWE = 1; //書き込み許可  1で許可
        //MPC.PE4PFS.BIT.PSEL = 0b001101; //SSLB0に設定
        MPC.PE5PFS.BIT.PSEL = 0b001101; //RSPCKBに設定
        MPC.PE6PFS.BIT.PSEL = 0b001101; //MOSIBに設定
        MPC.PE7PFS.BIT.PSEL = 0b001101; //MISOBに設定
        
        PORTE.PDR.BIT.B4 = 1;  //SSLB0はGPIOとしてCSを制御
        PORTE.PODR.BIT.B4 = 1;
        
        PORTA.PDR.BIT.B0 = 1;  //SSLA1のピンだったのでGPIOとしてCSを制御
        PORTA.PODR.BIT.B0 = 1;
        MPC.PWPR.BIT.PFSWE = 0; //PFSWEの書き込み禁止 0
        MPC.PWPR.BIT.B0WI = 1; //書き込み禁止

        //PORTE.PMR.BIT.B4 = 1; //ピンの設定をするときはまずピンを汎用ポートに設定しておく
        PORTE.PMR.BIT.B5 = 1;
        PORTE.PMR.BIT.B6 = 1;
        PORTE.PMR.BIT.B7 = 1;

        RSPI1.SPDCR.BIT.SPLW = 1; //SPDRレジスタへはロングアクセス

        //RSPIビットレートレジスタ(SPBR): 通常のシリアル通信と同様に設定する
        //ビットレート = f(PCLKA)/[2 × (n + 1) × 2^N]
        //mpu9250は1MHzまでOK
        //icm20608gは8MHzまでOK
        //ma730は25MHzまでOK
        RSPI1.SPBR = 1; // 24MHz
        //RSPI制御レジスタ(SPCR)
        RSPI1.SPCR.BIT.SPMS = 0;
        RSPI1.SPCR.BIT.TXMD = 0;

        //RSPIコマンドレジスタ(SPCMDx): 送受信フォーマットの設定
        RSPI1.SPCMD0.BIT.BRDV = 0;
        RSPI1.SPCMD0.BIT.CPHA = 1;
        RSPI1.SPCMD0.BIT.CPOL = 1;
        RSPI1.SPCMD0.BIT.SSLA = 3;
        RSPI1.SPCMD0.BIT.SSLKP = 1;
        RSPI1.SPDCR.BIT.SPFC = 0b00; //バッファのフレーム数を指定

        RSPI1.SPDCR.BIT.SPRDTD = 0; //受信バッファを読み出す
        RSPI1.SPCR.BIT.MSTR = 1;
        dummy = RSPI1.SPCR.BIT.MSTR;

    }

    void useSSLA0RSPI0() {
        setEnableRSPI0(0);
        PORTA.PODR.BIT.B4 = 1;
        RSPI0.SPBR = 4; //9.6MHz
        RSPI0.SPCMD0.BIT.SSLA = 0; //0:SSL0 1:SSL1
        RSPI0.SPCMD0.BIT.SPB = 0b0111;
        PORTA.PODR.BIT.B4 = 0;     
    }

    void useSSLA1SPI0() {

    }

    void useSSLA0RSPI1() {
        PORTA.PODR.BIT.B0 = 1;
        PORTE.PODR.BIT.B4 = 1;
        RSPI1.SPBR = 1; //24MHz
        RSPI1.SPCMD0.BIT.SPB = 0b1111;        
        PORTE.PODR.BIT.B4 = 0;
    }

    void useSSLA1RSPI1() {
        PORTA.PODR.BIT.B0 = 1;
        PORTE.PODR.BIT.B4 = 1;        
        RSPI1.SPBR = 1; //24MHz
        RSPI1.SPCMD0.BIT.SPB = 0b1111;
        PORTA.PODR.BIT.B0 = 0;
    }

    void setEnableRSPI0(uint8_t en) {
        RSPI0.SPCR.BIT.SPE = en;
    }

    void setEnableRSPI1(uint8_t en) {
        RSPI1.SPCR.BIT.SPE = en;
    }

    uint8_t communicate8bitRSPI0(uint8_t transmit) {
        setEnableRSPI0(1);
        unsigned int receive;
        RSPI0.SPDR.WORD.H = (uint8_t) transmit;
        while (RSPI0.SPSR.BIT.SPTEF != 1) { }
        while (RSPI0.SPSR.BIT.SPRF == 0) { }
        receive = RSPI0.SPDR.WORD.H;
        setEnableRSPI0(0);
        PORTA.PODR.BIT.B4 = 1;

        return (uint8_t) (receive & 0xff);
    }

    uint8_t communicate8bitRSPI1(uint8_t transmit) {
        setEnableRSPI1(1);
        unsigned int receive;
        RSPI1.SPDR.LONG = transmit;
        while (RSPI1.SPSR.BIT.SPTEF != 1) { }        
        while (RSPI1.SPSR.BIT.SPRF == 0){ }
        receive = RSPI1.SPDR.LONG;
        return (uint8_t) (receive & 0xff);
    }

    uint32_t communicate16bitRSPI1(uint16_t transmit) {        
        setEnableRSPI1(1);
        unsigned int receive;
        RSPI1.SPDR.LONG = transmit;
        while (RSPI1.SPSR.BIT.SPTEF != 1) { }
        while (RSPI1.SPSR.BIT.SPRF == 0){ }
        receive = RSPI1.SPDR.LONG;
        setEnableRSPI1(0);
        PORTA.PODR.BIT.B0 = 1;
        PORTE.PODR.BIT.B4 = 1;        
        return (uint16_t) (receive & 0xffffffff);        
    }

    void communicateNbyteRSPI0(uint8_t* send, uint8_t* recv, uint8_t num) {
        setEnableRSPI0(1);
        for (int i = 0; i < num; i++) {            
            RSPI0.SPDR.WORD.H = send[i];
            while (RSPI0.SPSR.BIT.SPTEF != 1) { }            
            while (RSPI0.SPSR.BIT.SPRF == 0) { }            
            recv[i] = RSPI0.SPDR.WORD.H;

        }
        setEnableRSPI0(0);
        PORTA.PODR.BIT.B4 = 1;
    }

    void communicateNbyteRSPI1(uint8_t* send, uint8_t* recv, uint8_t num) {

    }
}
