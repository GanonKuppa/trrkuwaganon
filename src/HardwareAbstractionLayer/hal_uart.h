#pragma once

#include <stdint.h>
#include <stdarg.h>
#include <deque>

namespace hal {
    void initUart0();
    void put1byteUart0(uint8_t c);
    void putnbyteUart0(uint8_t* buf, uint16_t len);
    
    
    void initUart1();
    void putnbyteUart1(uint8_t* buf, uint16_t len);
    void recvDataUart1();
    void sendDataUart1();
    bool readnbyteUart1(uint8_t* buf, uint16_t len);    
    bool isEmptyRecvBufUart1();
    uint16_t getRecvBufUart1size();

    int printfSync(const char* fmt, ...); // output to Uart0
    int printfAsync(const char* fmt, ...); // output to Uart1
    
    void initPickle();
    int printfPickle(const char* fmt, ...); // output to RAM
    int feedPickleWithPrintfAsync();
}



