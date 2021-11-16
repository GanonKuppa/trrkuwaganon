#include "shell.h"

#include <string.h>

// Lib/ntshell
#include "ntopt.h"
#include "ntlibc.h"
#include "ntshell.h"


#include "debugLog.h"

// Hal
#include "hal_uart.h"
#include "hal_timerInterrupt.h"
#include "hal_timer.h"

// Module
#include "baseModule.h"
#include "batteryMonitor.h"
#include "controlMixer.h"
#include "heater.h"
#include "imuDriver.h"
#include "ledController.h"
#include "logger.h"
#include "navigator.h"
#include "parameterManager.h"
#include "positionEstimator.h"
#include "powerTransmission.h"
#include "pseudoDial.h"
#include "shell.h"
#include "suction.h"
#include "trajectoryCommander.h"
#include "trajectoryInitializer.h"
#include "wallSensor.h"
#include "wheelOdometry.h"
#include "truthMaker.h"
#include "gamepad.h"
#include "seManager.h"

static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);
static int usrcmd_top(int argc, char **argv);

typedef int (*USRCMDFUNC)(int argc, char **argv);

typedef struct {
    const char *cmd;
    const char *desc;
    USRCMDFUNC func;
} cmd_table_t;



static const cmd_table_t cmdlist[] = {
    { "help", "help command.", usrcmd_help },
    { "info", "system info.", usrcmd_info },
    { "top", "top command.", usrcmd_top },
    { "batteryMonitor", "BatteryMonitor Module.", module::usrcmd_batteryMonitor },
    { "battery", "alias of batteryMonitor command.", module::usrcmd_batteryMonitor },
    { "controlMixer", "ControlMixer.", module::usrcmd_controlMixer },
    { "heater", "Heater Module.", module::usrcmd_heater },
    { "imuDriver", "ImuDriver Module.", module::usrcmd_imuDriver },
    { "imu", "alias of imuDriver command.", module::usrcmd_imuDriver },
    { "ledController", "LedController Module.", module::usrcmd_ledController },
    { "logger", "Logger Module.", module::usrcmd_logger },
    { "navigator", "Navigator Module.", module::usrcmd_navigator },
    { "paramManager", "ParamManager Module.", module::usrcmd_parameterManager },
    { "param", "alias of paramManager command.", module::usrcmd_parameterManager },    
    { "positionEstimator", "PositionEstimator Module.", module::usrcmd_positionEstimator },
    { "position", "alias of positionEstimator command.", module::usrcmd_positionEstimator },
    { "powerTransmission", "PowerTransmission Module.", module::usrcmd_powerTransmission },
    { "power", "alias of powerTransmission command.", module::usrcmd_powerTransmission },
    { "pseudoDial", "pseudoDial Module.", module::usrcmd_pseudoDial },
    { "shell", "shell Module.", module::usrcmd_shell },
    { "suction", "Suction Module.", module::usrcmd_suction },
    { "trajectoryCommander", "TrajectoryCommander Module.", module::usrcmd_trajectoryCommander },
    { "trajectoryInitializer", "TrajectoryInitializer Module.", module::usrcmd_trajectoryInitializer },
    { "wallSensor", "WallSensor Module.", module::usrcmd_wallSensor },
    { "wheelOdometry", "WheelOdometry Module.", module::usrcmd_wheelOdometry },
    { "wheel", "alias of wheelOdometry command.", module::usrcmd_wheelOdometry },
    { "truthMaker", "TruthMaker Module.", module::usrcmd_truthMaker },
    { "gamepad", "Gamepad Module.", module::usrcmd_gamepad },
    { "seManager", "seManager Module.", module::usrcmd_seManager }
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
        hal::waitmsec(10);
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

int usrcmd_top(int argc, char **argv)
{    
    float slot0_time = hal::getSlot0Time();
    float slot1_time = hal::getSlot1Time();
    float slot2_time = hal::getSlot2Time();
    float slot3_time = hal::getSlot3Time();
    PRINTF_ASYNC("\n");
    PRINTF_ASYNC("  --- timerInterrupt0 Slot Time 0, 1, 2, 3\n");
    PRINTF_ASYNC("      total = %6.2f[us]\n",slot0_time + slot1_time + slot2_time + slot3_time);
    PRINTF_ASYNC("      %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us]\n",slot0_time, slot1_time, slot2_time, slot3_time);    

    PRINTF_ASYNC("\n");

    PRINTF_ASYNC("  --- Modules Time --- \n");
    PRINTF_ASYNC("      update0   , update1   , update2   , update3   ,updateEvery, updateInMainLoop\n");
    
    module::BatteryMonitor::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::ControlMixer::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::Heater::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::ImuDriver::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::LedController::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::Logger::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::Navigator::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::ParameterManager::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::PositionEstimator::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::PowerTransmission::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::PseudoDial::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::Shell::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::Suction::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::TrajectoryCommander::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::TrajectoryInitializer::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::WallSensor::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::WheelOdometry::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::TruthMaker::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::Gamepad::getInstance().printCycleTime();
    hal::waitmsec(10);
    module::SeManager::getInstance().printCycleTime();
    hal::waitmsec(10);

    return 0;
}





namespace module {


    Shell::Shell(){
        setModuleName("Shell");
        void *extobj = 0;
        ntshell_init(&nts, serial_read, serial_write, user_callback, extobj);
        ntshell_set_prompt(&nts, "trrkuwaganon>");
    };

    void Shell::updateEvery(){
        hal::sendDataUart1();
        hal::recvDataUart1();
        hal::sendDataUart1();
    }
    void Shell::update0(){
        hal::sendDataUart1();
        hal::recvDataUart1();
        hal::sendDataUart1();
    }

    void Shell::update1(){
        hal::sendDataUart1();
        hal::recvDataUart1();
        hal::sendDataUart1();
    }

    void Shell::update2(){
        hal::sendDataUart1();
        hal::recvDataUart1();
        hal::sendDataUart1();
    }

    void Shell::update3(){
        hal::sendDataUart1();
        hal::recvDataUart1();
        hal::sendDataUart1();
    }

    void Shell::updateInMainLoop(){
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


    int Shell::serial_read(char *buf, int cnt, void *extobj)
    {    

        if(!hal::isEmptyRecvBufUart1()){
            uint16_t recvBufsize = hal::getRecvBufUart1size();
            uint16_t read_size = cnt;
            if(cnt > recvBufsize){
                read_size = recvBufsize;
            }
            bool rtn = hal::readnbyteUart1((uint8_t*)buf, read_size);
            
            if(rtn == false){
                return 0;
            } 
            else{
                return read_size;
            }
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


    int usrcmd_shell(int argc, char **argv){
    	return 0;
    }


}






