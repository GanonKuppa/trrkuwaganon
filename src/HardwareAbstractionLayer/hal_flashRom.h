#pragma once
#include <stdint.h>

namespace hal {
    constexpr uint32_t FLASH_BLOCK_NUM = 1024;
    constexpr uint32_t FLASH_BLOCK_BYTE_SIZE = 64;
    constexpr uint32_t FLASH_ROM_SIZE = 65535;

    bool initFlashRom();
    uint8_t readFlashRom(uint32_t org);
    bool readFlashRom(uint32_t org, void* dst, uint32_t len);
    bool eraseCheckFlashRom(uint32_t org, uint32_t len);
    bool eraseFlashRom(uint32_t org);
    bool eraseAllFlashRom();
    bool writeFlashRom(uint32_t org, const void* src, uint32_t len);
    bool writeFlashRom(uint32_t org, uint8_t data);
#ifdef SILS
    void sils_flashrom_debug();
#endif
}