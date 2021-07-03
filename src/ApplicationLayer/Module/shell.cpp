#include "shell.h"

// Lib/ntshell
#include "ntopt.h"
#include "ntlibc.h"
#include "ntshell.h"
#include "debugLog.h"
#include "string.h"
#include "hal_uart.h"


static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);

typedef int (*USRCMDFUNC)(int argc, char **argv);

typedef struct {
    char *cmd;
    char *desc;
    USRCMDFUNC func;
} cmd_table_t;



static const cmd_table_t cmdlist[] = {
    { "help", "help_command", usrcmd_help },
    { "info", "system info.", usrcmd_info },
    { "batteryVoltageMonitor", "BatteryVoltageMonitor Module", usrcmd_info },
    { "communication", "Communication Module", usrcmd_info },
    { "gamepad", "Gamepad Module", usrcmd_info },
    { "heater", "Heater Module", usrcmd_info },
    { "ledController", "LedController Module", usrcmd_info },
    { "suction", "Suction Module", usrcmd_info },
    { "wallSensor", "WallSensor Module", usrcmd_info },
    { "imuDriver", "ImuDriver Module", usrcmd_info },
    { "paramManager", "ParamManager Module", usrcmd_info },
    { "param", "alias of paramManager command", usrcmd_info }
};

int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        PRINTF_ASYNC("  ");
        PRINTF_ASYNC(p->cmd);
        int command_len_max = 24;
        int space_num = command_len_max - strlen(p->cmd);
        if(space_num > 0){
            for(int i = 0; i < space_num; i++){
                PRINTF_ASYNC(" ");
            }
        }
        PRINTF_ASYNC(": ");
        PRINTF_ASYNC(p->desc);
        PRINTF_ASYNC("\r\n");
        p++;
    }
    return 0;
}

int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
        PRINTF_ASYNC("info sys\r\n");
        PRINTF_ASYNC("info ver\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
        PRINTF_ASYNC("  ");
        PRINTF_ASYNC("trrkuwaganon\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
        PRINTF_ASYNC("  ");
        PRINTF_ASYNC("Version 0.0.1\r\n");
        return 0;
    }
    PRINTF_ASYNC("  ");
    PRINTF_ASYNC("Unknown sub command found\r\n");
    return -1;
}





namespace module {


    Shell::Shell(){
        void *extobj = 0;
        ntshell_init(&nts, serial_read_1byte, serial_write, user_callback, extobj);
        ntshell_set_prompt(&nts, "trrkuwaganon>");
    };

    void Shell::update(){
        hal::recvDataUart1();
        hal::sendDataUart1();

        ntshell_execute(&nts);
    }


    int Shell::usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
    {
        if (argc == 0) {
            return 0;
        }
        const cmd_table_t *p = &cmdlist[0];
        for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
            if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
                return p->func(argc, argv);
            }
            p++;
        }
        PRINTF_ASYNC("Unknown command found.\r\n");
        return 0;
    }

    int Shell::user_callback(const char *text, void *extobj)
    {    
        ntopt_parse(text, usrcmd_ntopt_callback, 0);
        return 0;
    }


    int Shell::serial_read_1byte(char *buf, int cnt, void *extobj)
    {    

        if(!hal::isEmptyRecvBufUart1() && hal::readnbyteUart1((uint8_t*)buf, 1)){
            return 1;
        }
        else{
            return 0;
        }
    }

    int Shell::serial_write(const char *buf, int cnt, void *extobj)
    {
        hal::putnbyteUart1((uint8_t*)buf, cnt);
        return cnt;
    }


}






