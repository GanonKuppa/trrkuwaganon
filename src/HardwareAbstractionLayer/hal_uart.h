#pragma once

#include <stdint.h>
#include <deque>

namespace hal {

    void initUart0();
    void put1byteUart0(uint8_t c);
    void putnbyteUart0(uint8_t* buf, uint16_t len);

    void initUart1();
    void putnbyteUart1(uint8_t* buf, uint16_t len);
    void recvDataUart1();
    void sendDataUart1();

    std::deque<uint8_t>& getSendBufUart1();
    std::deque<uint8_t>& getRecvBufUart1();
}



