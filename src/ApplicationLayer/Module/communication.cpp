#include <stdarg.h>
#include <stdio.h>
#include "communication.h"
#include "hal_uart.h"



namespace module {

    Communication::Communication(){};

    void Communication::update(){ 

    }

    int Communication::printfAsync(const char* fmt, ...){
        int len = 0;

        va_list ap;
        va_start(ap, fmt);
        static char buffer[1000];
        len = vsprintf(buffer, fmt, ap);

#ifndef SILS
        hal::putnbyteUart0((uint8_t*)buffer, len);
#else
        printf("%s", buffer);
#endif
        va_end(ap);
        return len;
    }    
    

    int Communication::printfSync(const char* fmt, ...){
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
