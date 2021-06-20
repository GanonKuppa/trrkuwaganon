#include <stdint.h>
#include "hal_flashRom.h"

#ifndef SILS
#include "pd_flashRom.h"
#else
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#endif

namespace hal {


#ifndef SILS

#else
    static const char ROM_PATH[]   = "sils_flashRom.bin";

    static bool existFile(const char* path)
    {
        FILE* fp = fopen(ROM_PATH, "r");
        if (fp == NULL) {
            return false;
        }

        fclose(fp);
        return true;
    }


    static long long getFileSize(const char* path)  {
        long long size = 0;
        if(!existFile(ROM_PATH)) return -1; 

        FILE* fp = fopen(ROM_PATH, "rb");
        if (fp == NULL) {
            return -1;
        }

        fseek( fp, 0, SEEK_END );
        size = ftell(fp);
        fclose(fp);

        return size;
    }


#endif



    bool initFlashRom() {
#ifndef SILS
        return periferal_driver::initDataFlash();
#else
        
        bool is_file_exist = existFile(ROM_PATH);
        long long size = getFileSize(ROM_PATH);
        
        if(!is_file_exist || size < FLASH_ROM_SIZE){
            FILE* fp = fopen(ROM_PATH, "wb");
            if (fp == NULL) {
                return false;
            }
            
            if (fp != NULL) {
                for(uint32_t i=0;i<FLASH_ROM_SIZE;i++){
                    uint8_t c = 0;
                    if( fwrite( &c, sizeof(c), 1, fp ) < 1 ){
                        printf( "ファイルへの書き込みに失敗しました。\n");            
                        return false;
                    }
                }        
            }
            fclose(fp);        
        }
        return true;
#endif
    }

    uint8_t readFlashRom(uint32_t org) {
#ifndef SILS
        return periferal_driver::readDataFlash(org);
#else
        FILE* fp = fopen(ROM_PATH, "rb");
        if (fp == NULL) {
            return false;
        }
        fseek(fp, org, SEEK_SET);
        uint8_t dst;
        fread(&dst, 1, 1, fp);
        fseek( fp, 0, SEEK_END );
        fclose(fp);

        return dst;
#endif
    }

    bool readFlashRom(uint32_t org, void* dst, uint32_t len) {
#ifndef SILS
        return periferal_driver::readDataFlash(org, dst, len);
#else
        FILE* fp = fopen(ROM_PATH, "rb");
        if (fp == NULL) {
            return false;
        }
        fseek(fp, org, SEEK_SET);
        fread(dst, 1, len, fp);
        fseek( fp, 0, SEEK_END );
        fclose(fp);
        return true;
#endif
    }

    bool eraseCheckFlashRom(uint32_t org, uint32_t len) {
#ifndef SILS
        return periferal_driver::eraseCheckDataFlash(org, len);
#else
        return true;
#endif
    }

    bool eraseFlashRom(uint32_t org) {
#ifndef SILS
        return periferal_driver::eraseDataFlash(org);
#else
        return true;
#endif
    }

    bool eraseAllFlashRom() {
#ifndef SILS
        return periferal_driver::eraseAllDataFlash();
#else
        FILE* fp = fopen( "sils_flashRom.bin", "wb" );
        if( fp == NULL ){
            printf( "ファイルオープンに失敗しました。\n");        
        }

        for(uint32_t i=0;i<65535;i++){
            uint8_t c = 0;
            if( fwrite( &c, sizeof(c), 1, fp ) < 1 ){
                printf( "ファイルへの書き込みに失敗しました。\n");            
            }
        }
        fclose(fp);
        return true;
#endif
    }

    bool writeFlashRom(uint32_t org, const void* src, uint32_t len) {
#ifndef SILS
        return periferal_driver::writeDataFlash(org, src, len);
#else
        uint8_t read_data[FLASH_ROM_SIZE];
        readFlashRom(0, read_data, FLASH_ROM_SIZE);
        
        memcpy(&read_data[org], src ,len);

        FILE* fp = fopen(ROM_PATH, "wb");
        if (fp == NULL) {
            return false;
        }

        fwrite(read_data, FLASH_ROM_SIZE, 1, fp);
        fclose(fp);
        return true;
#endif
    }

    bool writeFlashRom(uint32_t org, uint8_t data) {
#ifndef SILS
        return periferal_driver::writeDataFlash(org, data);
#else
        uint8_t read_data[FLASH_ROM_SIZE];
        readFlashRom(0, read_data, FLASH_ROM_SIZE);
        read_data[org] = data;
        FILE* fp = fopen(ROM_PATH, "wb");
        if (fp == NULL) {
            return false;
        }

        fwrite(read_data, FLASH_ROM_SIZE, 1, fp);
        fclose(fp);
        return true;
#endif
    }

#ifdef SILS
    void sils_flashrom_debug(){
                        
        printf("----------------\n");

        uint8_t data[FLASH_ROM_SIZE];
        readFlashRom(0, data, 1024);

        for(int i=0; i<64;i++){
            printf("%08x   ", i*16);
            for(int j=0;j<16;j++){
                printf("%02x ", data[i*16 +j]);
            }
            printf("\n");
        }        


        for(int i=0; i<5;i++){
            writeFlashRom(i, i);
            printf("%d %d \n",i, readFlashRom(i));
        }
        uint8_t hoge[10] = {0,10,20,30,40,50,60,70,80,90};
        writeFlashRom(16, hoge, 10);
                
        printf("----------------\n");     
        readFlashRom(0, data, 1024);   
        for(int i=0; i<64;i++){
            printf("%08x   ", i*16);
            for(int j=0;j<16;j++){
                printf("%02x ", data[i*16 +j]);
            }
            printf("\n");
        }        

    }
#endif



}
