#pragma once

#include <stdint.h>
#include <deque>


namespace periferal_driver {

    void initSCI1();
    void put1byteSCI1(uint8_t c);
    void putnbyteSCI1(uint8_t* buf, uint16_t len);

    void initSCIFA9();
    void putnbyteSCIFA9(uint8_t* buf, uint16_t len);
    void recvDataSCIFA9();
    void sendDataSCIFA9();

    std::deque<uint8_t>& getSendBufSCIFA9();
    std::deque<uint8_t>& getRecvBufSCIFA9();

}
