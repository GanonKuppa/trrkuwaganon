#pragma once

#include <stdint.h>
#include <deque>


namespace peripheral_driver {

    void initSCI1();
    void put1byteSCI1(uint8_t c);
    void putnbyteSCI1(uint8_t* buf, uint16_t len);

    void initSCIFA9();
    void put1byteSCIFA9(uint8_t c);
    void putnbyteSCIFA9(uint8_t* buf, uint16_t len);
    bool readnbyteSCIFA9(uint8_t* buf, uint16_t len);
    bool isEmptyRecvBufSCIFA9();
    uint16_t getSCIFA9Bufsize();
    void recvDataSCIFA9();
    void sendDataSCIFA9();
}
