#include <stdint.h>
#include <cstring>

#include "pd_flashRom.h"
#include "iodefine.h"


//メモリ構成
//RX71Mのデータフラッシュは64byteがワンブロック
//データ消去はブロック単位で行う
//64byteのブロックが1024個存在する(ブロック0 - ブロック1023)
//アドレスは  0010 0000h ブロック0
//       0010 003Fh ブロック1
//       0010 FFFFh ブロック1023

//=====================================================================//
/*! @file
 @brief  RX64M/RX71M グループ FLASH 制御 @n
 ・データフラッシュ消去サイズ（６４バイト単位）
 ・データフラッシュ書き込みサイズ（４バイト単位）
 @author 平松邦仁 (hira@rvf-rc45.net)
 @copyright  Copyright (C) 2017 Kunihito Hiramatsu @n
 Released under the MIT license @n
 https://github.com/hirakuni45/RX/blob/master/LICENSE
 */
//=====================================================================//

namespace periferal_driver {

    static const uint32_t data_flash_block = 64; // データ・フラッシュのブロックサイズ
    static const uint32_t data_flash_size = 65536; // データ・フラッシュの容量
    static const uint32_t data_flash_bank = 1024; // データ・フラッシュのバンク数
    static const uint32_t F_FCLK = 48000000;

    enum class error
        : uint8_t {
        NONE, //< エラー無し
        ADDRESS, //< アドレス・エラー
        TIMEOUT, //< タイム・アウト・エラー
        LOCK, //< ロック・エラー
    };

    enum class mode
        : uint8_t {
        NONE, RD, PE
    };

    static error error_ = error::NONE;
    static mode mode_ = mode::NONE;
    static bool trans_farm_ = false;

    //8bitフリーランニングカウンタとして動作
    //TMR1をデータフラッシュ書き込み時の待ち時間生成に利用
    static const unsigned int u_count_TMR1 = 6;
    void initTMR1() {
        SYSTEM.PRCR.WORD = 0xA502;
        SYSTEM.MSTPCRA.BIT.MSTPA5 = 0; //TMR1と1 ON
        SYSTEM.PRCR.WORD = 0xA500;

        TMR1.TCCR.BIT.CSS = 1; //1:PCLKBをカウントソースに設定
        TMR1.TCCR.BIT.CKS = 0; //0:分周比1
    }

    void waitusec_TMR1(uint8_t usec) {

        TMR1.TCNT = 0;
        TMR1.TCSTR.BIT.TCS = 1; //カウント開始
        while (1) {
            if ( TMR1.TCNT >= u_count_TMR1 * usec) break;
        }

        TMR1.TCSTR.BIT.TCS = 0; //カウント停止

    }

    // FACIコマンド発行領域 007E 0000h 4バイト

    //FACI強制終了コマンドの発行
    static bool FACI_termination() {
        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0xB3;

        uint32_t cnt = 22;
        if (F_FCLK < 20000000) cnt = 36;

        while (FLASH.FSTATR.BIT.FRDY == 0) {
            waitusec_TMR1(1);
            --cnt;
            if (cnt == 0) break;
        }
        if (cnt == 0) {
            //debug_format("FACI 'turn_break_' timeout\n");
            return false;
        }

        if (FLASH.FASTAT.BIT.CMDLK == 0) {
            return true;
        } else {
            //debug_format("FACI 'turn_break_' fail\n");
            return false;
        }

    }

    //データフラッシュリードモード移行
    static void FACI_readMD() {
        uint32_t n = 5;

        while (FLASH.FSTATR.BIT.FRDY == 0) {
            waitusec_TMR1(1);
            --n;
            if (n == 0) break;
        }

        if (n == 0 || FLASH.FASTAT.BIT.CMDLK != 0) {
            FACI_termination();

        }

        FLASH.FENTRYR.WORD = 0xAA00;

        if (FLASH.FENTRYR.WORD != 0x0000) {
            //debug_format("FACI 'turn_rd_' fail\n");
        }
        mode_ = mode::RD;
    }

    //PEモードへ移行
    static void FACI_peMD() {
        FLASH.FENTRYR.WORD = 0xAA80;

        if (FLASH.FENTRYR.WORD != 0x0080) {
            //debug_format("FACI 'turn_pe_' fail\n");
        }
        mode_ = mode::PE;
    }

    //FCUの初期化 FCU= Flash Control Unit
    static bool initFCU() {
        if (trans_farm_) return true;

        if (FLASH.FENTRYR.WORD != 0) {
            FLASH.FENTRYR.WORD = 0xAA00;

            uint32_t wait = 4;
            while (FLASH.FENTRYR.WORD != 0) {
                if (wait > 0) {
                    --wait;
                } else {
                    //debug_format("FACI Tras FARM timeout\n");
                    return false;
                }
            }
        }

        FLASH.FCURAME.WORD = 0xC403; //Write onlyらしい

        const uint32_t* src = reinterpret_cast<const uint32_t*>(0xFEFFF000); // Farm master
        uint32_t* dst = reinterpret_cast<uint32_t*>(0x007F8000); // Farm section
        for (uint32_t i = 0; i < (4096 / 4); ++i) {
            *dst++ = *src++;
        }

        FLASH.FCURAME.WORD = 0xC400;
        FACI_peMD();

        auto f = FACI_termination();
        if (f) {
            FACI_readMD();
            trans_farm_ = true;
        } else {
            FACI_termination();
            //debug_format("FACI Tras FARM lock\n");
        }
        return trans_farm_;
    }

    // 4 バイト書き込み
    // org: align 4 bytes
    // 書き込みは4byte単位!
    static bool write32(const void* src, uint32_t org) {
        FLASH.FPROTR.WORD = 0x5501;
        FLASH.FSADDR.LONG = org;

        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0xE8;
        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0x02;

        const uint8_t* p = static_cast<const uint8_t*>(src);
        *reinterpret_cast<volatile uint16_t*>(0x007E0000) = (static_cast<uint16_t>(p[1]) << 8)
                | static_cast<uint16_t>(p[0]);

        while (FLASH.FSTATR.BIT.DBFULL != 0) {
            asm("nop");
        }

        *reinterpret_cast<volatile uint16_t*>(0x007E0000) = (static_cast<uint16_t>(p[3]) << 8)
                | static_cast<uint16_t>(p[2]);

        while (FLASH.FSTATR.BIT.DBFULL != 0) {
            asm("nop");
        }

        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0xD0;

        // write (4 bytes): FCLK 20MHz to 60MHz max 1.7ms
        //                  FCLK 4MHz max 3.8ms
        // * 1.1
        uint32_t cnt = 1870;
        if (F_FCLK < 20000000) cnt = 4180;

        while (FLASH.FSTATR.BIT.FRDY == 0) {
            waitusec_TMR1(1);
            --cnt;
            if (cnt == 0) break;
        }
        if (cnt == 0) { // time out
            FACI_termination();
            error_ = error::TIMEOUT;
            //debug_format("FACI 'write32_' timeout\n");
            return false;
        }

        if (FLASH.FASTAT.BIT.CMDLK != 0) {
            error_ = error::LOCK;
            //debug_format("FACI 'write32_' CMD Lock fail\n");
            return false;
        }

        error_ = error::NONE;
        return true;
    }

    bool initDataFlash() {
        if (trans_farm_) return false; // ファームが既に転送済み
        initTMR1();
        FLASH.FWEPROR.BIT.FLWE = 1;

        uint32_t clk = static_cast<uint32_t>(F_FCLK) / 500000;
        if (clk & 1) {
            clk >>= 1;
            ++clk;
        } else {
            clk >>= 1;
        }
        if (clk > 60) {
            clk = 60;
        }
        FLASH.FPCKAR.WORD = 0x1E00 | clk;

        initFCU();

        error_ = error::NONE;

        return true;
    }

    uint8_t readDataFlash(uint32_t org) {
        if (org >= data_flash_size) {
            error_ = error::ADDRESS;
            return 0;
        }

        if (mode_ != mode::RD) {
            FACI_readMD();
        }

        error_ = error::NONE;

        uint8_t read_val = *reinterpret_cast<volatile uint8_t*>(0x00100000 + org);
        return read_val;
    }

    bool readDataFlash(uint32_t org, void* dst, uint32_t len) {
        if (org >= data_flash_size) {
            error_ = error::ADDRESS;
            return false;
        }
        if ((org + len) > data_flash_size) {
            len = data_flash_size - org;
        }
        if (mode_ != mode::RD) {
            FACI_readMD();
        }

        const void* src = reinterpret_cast<const void*>(0x00100000 + org);
        std::memcpy(dst, src, len);

        error_ = error::NONE;

        return true;
    }

    bool eraseCheckDataFlash(uint32_t org, uint32_t len) {
        if (org >= data_flash_size) {
            error_ = error::ADDRESS;
            return false;
        }

        if (mode_ != mode::PE) {
            FACI_peMD();
        }

        FLASH.FBCCNT.BYTE = 0x00;
        FLASH.FSADDR.LONG = org;
        FLASH.FEADDR.LONG = org + len - 1;

        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0x71;
        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0xD0;

        // erase cheak (4 bytes): FCLK 20MHz to 60MHz max 30us
        //                        FCLK 4MHz max 84us
        // * 1.1
        uint32_t cnt = 33 * 64 / 4;
        if (F_FCLK < 20000000) cnt = 93 * 64 / 4;
        while (FLASH.FSTATR.BIT.FRDY == 0) {

            waitusec_TMR1(1);
            --cnt;
            if (cnt == 0) break;
        }
        if (cnt == 0) { // time out
            FACI_termination();
            error_ = error::TIMEOUT;
            //debug_format("FACI 'erase_check' timeout\n");
            return false;
        }

        if (FLASH.FASTAT.BIT.CMDLK == 0) {
            return (FLASH.FBCSTAT.BIT.BCST == 0);
        } else {
            error_ = error::LOCK;
            //debug_format("FACI 'erase_check' lock fail\n");
            return false;
        }
    }

    bool eraseDataFlash(uint32_t org) {
        if (org >= data_flash_size) {
            error_ = error::ADDRESS;
            return false;
        }

        if (mode_ != mode::PE) {
            FACI_peMD();
        }

        FLASH.FPROTR.WORD = 0x5501; // ロックビットプロテクト無効
        FLASH.FCPSR.WORD = 0x0000; // サスペンド優先
        FLASH.FSADDR.LONG = org;

        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0x20;
        *reinterpret_cast<volatile uint8_t*>(0x007E0000) = 0xD0;

        // 64 bytes erase: FCLK 20MHz to 60MHz max 10ms
        //                 FCLK 4MHz max 18ms
        // * 1.1
        uint32_t cnt = 1100;
        if (F_FCLK < 20000000) cnt = 1980;
        while (FLASH.FSTATR.BIT.FRDY == 0) {
            waitusec_TMR1(1);
            --cnt;
            if (cnt == 0) break;
        }

        if (cnt == 0) { // time out
            FACI_termination();
            error_ = error::TIMEOUT;
            //debug_format("FACI 'erase' timeout\n");
            return false;
        }

        if (FLASH.FASTAT.BIT.CMDLK == 0) {

            error_ = error::NONE;
            return true;
        } else {
            error_ = error::LOCK;
            //debug_format("FACI 'erase' lock fail\n");
            return false;
        }
    }

    bool eraseAllDataFlash() {
        for (uint32_t pos = 0; pos < data_flash_size; pos += data_flash_block) {
            if (!eraseCheckDataFlash(pos, data_flash_block)) {
                auto ret = eraseDataFlash(pos);
                if (!ret) {
                    return false;
                }
            }
        }
        return true;
    }

    bool writeDataFlash(uint32_t org, const void* src, uint32_t len) {
        if (org >= data_flash_size) {
            error_ = error::ADDRESS;
            return false;
        }

        if ((org + len) > data_flash_size) {
            len = data_flash_size - org;
        }

        if (mode_ != mode::PE) {
            FACI_peMD();
        }

        const uint8_t* p = static_cast<const uint8_t*>(src);
        bool f = false;
        int32_t l = static_cast<int32_t>(len);
        while (l > 0) {
            uint32_t mod = org & 3;
            if (mod != 0) {
                uint8_t tmp[4];
                tmp[0] = 0xFF;
                tmp[1] = 0xFF;
                tmp[2] = 0xFF;
                tmp[3] = 0xFF;
                l -= 4 - mod;
                while (mod < 3) {
                    tmp[mod] = *p++;
                    ++mod;
                }
                org &= 0xFFFFFFFC;
                f = write32(tmp, org);
            } else {
                f = write32(p, org);
                p += 4;
                l -= 4;
            }
            if (!f) break;
            org += 4;
        }

        if (f) {
            error_ = error::NONE;
        }

        return f;
    }

    bool writeDataFlash(uint32_t org, uint8_t data) {
        uint8_t d = data;
        return writeDataFlash(org, &d, 1);
    }

}