/**
 * @file usrcmd.c
 * @author CuBeatSystems
 * @author Shinichiro Nakamura
 * @copyright
 * ===============================================================
 * Natural Tiny Shell (NT-Shell) Version 0.3.1
 * ===============================================================
 * Copyright (c) 2010-2016 Shinichiro Nakamura
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "ntopt.h"
#include "ntlibc.h"
#include "debugLog.h"
#include "string.h"

typedef int (*USRCMDFUNC)(int argc, char **argv);

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);

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

int usrcmd_execute(const char *text)
{
    return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
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

static int usrcmd_help(int argc, char **argv)
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

static int usrcmd_info(int argc, char **argv)
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

