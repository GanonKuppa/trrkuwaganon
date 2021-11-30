#pragma once

#include <stdint.h>

namespace peripheral_driver {
    void initRSPI0();
    void initRSPI1();

    void useSSLA0RSPI0();
    void useSSLA1RSPI0();
    
    void useSSLA0RSPI1();
    void useSSLA1RSPI1();

    void setEnableRSPI0(uint8_t en);
    void setEnableRSPI1(uint8_t en);

    uint8_t communicate8bitRSPI0(uint8_t transmit);
    uint8_t communicate8bitRSPI1(uint8_t transmit);
    
    uint32_t communicate16bitRSPI1(uint16_t transmit);

    void communicateNbyteRSPI0(uint8_t* send, uint8_t* recv, uint8_t num);
    void communicateNbyteRSPI1(uint8_t* send, uint8_t* recv, uint8_t num);

}
