#include <stdint.h>
#include <stdio.h>
#include <deque>

#include "hal_uart.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_uart.h>
#endif


namespace hal {
    void initUart0() {
#ifndef SILS
        peripheral_driver::initSCI1();
#endif
    }

    void put1byteUart0(uint8_t c) {
#ifndef SILS
        peripheral_driver::put1byteSCI1(c);
#endif
    }

    void putnbyteUart0(uint8_t* buf, uint16_t len) {
#ifndef SILS
        peripheral_driver::putnbyteSCI1(buf, len);
#endif
    }

    void initUart1() {
#ifndef SILS
        peripheral_driver::initSCIFA9();
#endif
    }

    void putnbyteUart1(uint8_t* buf, uint16_t len) {
#ifndef SILS
        peripheral_driver::putnbyteSCIFA9(buf, len);
#endif
    }

    void recvDataUart1() {
#ifndef SILS
        peripheral_driver::recvDataSCIFA9();
#endif
    }

    void sendDataUart1() {
#ifndef SILS
        peripheral_driver::sendDataSCIFA9();
#endif
    }

    bool readnbyteUart1(uint8_t* buf, uint16_t len){
#ifndef SILS
        bool rtn = peripheral_driver::readnbyteSCIFA9(buf, len);
        return rtn;
#endif
        return false;
    }

    bool isEmptyRecvBufUart1(){
#ifndef SILS
        return peripheral_driver::isEmptyRecvBufSCIFA9();
#endif
        return false;
    }
    
    uint16_t getRecvBufUart1size(){
#ifndef SILS
        uint16_t size = peripheral_driver::getSCIFA9Bufsize();
        return size;
#endif
        return 0;
    };


    int printfAsync(const char* fmt, ...){
        int len = 0;

        va_list ap;
        va_start(ap, fmt);
        static char buffer[1000];
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

    constexpr int PICKLE_BUF_MAX = 4096;
    static std::deque<uint8_t> _pickleBuf(PICKLE_BUF_MAX);//?????????????????????????????????
    static uint8_t _lock_guard_pickle = false;

    void initPickle(){
        //_pickleBuf.reserve(PICKLE_BUF_MAX);
    }

    int printfPickle(const char* fmt, ...){
        int len = 0;

        va_list ap;
        va_start(ap, fmt);
        static char buffer[512];
        len = vsprintf(buffer, fmt, ap);
        _lock_guard_pickle = true;
        for (uint16_t i = 0; i < len; i++) {
            if(_pickleBuf.size() < PICKLE_BUF_MAX){
                _pickleBuf.push_back(buffer[i]);
            }
            else{
                if(!_pickleBuf.empty())_pickleBuf.pop_front();
                _pickleBuf.push_back(buffer[i]);
            }
        }
        _lock_guard_pickle = false;
        va_end(ap);
        return len;
    }

    int feedPickleWithPrintfAsync(){
#ifndef SILS
        for(auto &c : _pickleBuf){            
            peripheral_driver::put1byteSCIFA9(c);
            if(peripheral_driver::getSCIFA9Bufsize() > 512){
                while(peripheral_driver::getSCIFA9Bufsize() > 256);
            }
        }
#endif
        return 0;
    };

}
