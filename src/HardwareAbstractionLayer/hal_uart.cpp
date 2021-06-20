#include <stdint.h>
#include <deque>

#include "hal_uart.h"

#ifndef SILS
#include "pd_uart.h"
#endif

using std::deque;

namespace hal {
    void initUart0() {
#ifndef SILS
        periferal_driver::initSCI1();
#endif
    }

    void put1byteUart0(uint8_t c) {
#ifndef SILS
        periferal_driver::put1byteSCI1(c);
#endif
    }

    void putnbyteUart0(uint8_t* buf, uint16_t len) {
#ifndef SILS
        periferal_driver::putnbyteSCI1(buf, len);
#endif
    }

    void initUart1() {
#ifndef SILS
        periferal_driver::initSCIFA9();
#endif
    }

    void putnbyteUart1(uint8_t* buf, uint16_t len) {
#ifndef SILS
        periferal_driver::putnbyteSCIFA9(buf, len);
#endif
    }

    void recvDataUart1() {
#ifndef SILS
        periferal_driver::recvDataSCIFA9();
#endif
    }

    void sendDataUart1() {
#ifndef SILS
        periferal_driver::sendDataSCIFA9();
#endif
    }


    static std::deque<uint8_t> dummy_sendBuf; //送信用データバッファ
    static std::deque<uint8_t> dummy_recvBuf;//受信用データバッファ


    std::deque<uint8_t>& getSendBufUart1() {
#ifndef SILS
        return periferal_driver::getSendBufSCIFA9();
#else
        return dummy_sendBuf;
#endif
    }

    std::deque<uint8_t>& getRecvBufUart1() {
#ifndef SILS
        return periferal_driver::getRecvBufSCIFA9();
#else
        return dummy_recvBuf;
#endif
    }

}
