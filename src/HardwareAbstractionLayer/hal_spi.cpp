#include <stdint.h>
#include "hal_spi.h"

#ifndef SILS
#include "pd_spi.h"
#endif
namespace hal {
    void initSPI0() {
#ifndef SILS
        periferal_driver::initRSPI0();
#endif
    }

    void initSPI1() {
#ifndef SILS
        periferal_driver::initRSPI1();
#endif
    }

    void useCS0SPI0() {
#ifndef SILS
        periferal_driver::useSSLA0RSPI0();
#endif
    }

    void useCS0SPI1() {
#ifndef SILS
        periferal_driver::useSSLA0RSPI1();
#endif
    }

    void useCS1SPI1() {
#ifndef SILS
        periferal_driver::useSSLA1RSPI1();
#endif
    }

    void setEnableSPI0(uint8_t en) {
#ifndef SILS
        periferal_driver::setEnableRSPI0(en);
#endif
    }

    void setEnableSPI1(uint8_t en) {
#ifndef SILS
        periferal_driver::setEnableRSPI1(en);
#endif
    }

    uint8_t communicate8bitSPI0(uint8_t transmit) {
#ifndef SILS
        return periferal_driver::communicate8bitRSPI0(transmit);
#else
        return 0;
#endif
    }

    uint8_t communicate8bitSPI1(uint8_t transmit) {
#ifndef SILS
        return periferal_driver::communicate8bitRSPI1(transmit);
#else
        return 0;
#endif

    }

    uint32_t communicate16bitSPI1(uint16_t transmit) {
#ifndef SILS
        return periferal_driver::communicate16bitRSPI1(transmit);
#else
        return 0;
#endif
    }



    void communicateNbyteSPI0(uint8_t* send, uint8_t* recv, uint8_t num) {
#ifndef SILS
        periferal_driver::communicateNbyteRSPI0(send, recv, num);
#endif
    }

    void communicateNbyteSPI1(uint8_t* send, uint8_t* recv, uint8_t num) {
#ifndef SILS
        periferal_driver::communicateNbyteRSPI1(send, recv, num);
#endif
    }

}
