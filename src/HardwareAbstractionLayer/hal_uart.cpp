#include <stdint.h>
#include <stdio.h>
#include <deque>

#include "hal_uart.h"

#ifndef SILS
#include "pd_uart.h"
#endif


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

    bool readnbyteUart1(uint8_t* buf, uint16_t len){
#ifndef SILS
        bool rtn = periferal_driver::readnbyteSCIFA9(buf, len);
        return rtn;
#endif
        return false;
    }

    bool isEmptyRecvBufUart1(){
#ifndef SILS
        periferal_driver::isEmptyRecvBufSCIFA9();
#endif
        return false;
    }
    
    int printfAsync(const char* fmt, ...){
        int len = 0;

        va_list ap;
        va_start(ap, fmt);
        static char buffer[512];
        len = vsprintf(buffer, fmt, ap);

#ifndef SILS
        hal::putnbyteUart1((uint8_t*)buffer, len);
#else
        printf("%s", buffer);
#endif
        va_end(ap);
        return len;
    }    
    

    int printfSync(const char* fmt, ...){
        int len = 0;

        va_list ap;
        va_start(ap, fmt);
        static char buffer[1000];
        len = vsprintf(buffer, fmt, ap);

#ifndef SILS
        hal::putnbyteUart0((uint8_t*)buffer, len);
#else
        len = printf("%s", buffer);
#endif
        va_end(ap);
        return len;
    }





}
