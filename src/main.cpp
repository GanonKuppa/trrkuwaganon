#include <iodefine.h>
#include <stdio.h>
#include <stdarg.h>

// HardwareAbstractionLayer
#include "hal_clock.h"
#include "hal_gpio.h"
#include "hal_timer.h"
#include "hal_uart.h"
#include "hal_ad.h"
#include "hal_da.h"
#include "hal_flashRom.h"
#include "hal_phaseCounting.h"
#include "hal_pwm.h"
#include "hal_spi.h"
#include "hal_timerInterrupt.h"
#include "hal_wdt.h"

// Module
#include "batteryMonitor.h"
#include "controlMixer.h"
#include "gamepad.h"
#include "heater.h"
#include "imuDriver.h"
#include "ledController.h"
#include "navigator.h"
#include "parameterManager.h"
#include "positionEstimator.h"
#include "powerTransmission.h"
#include "pseudoDial.h"
#include "seManager.h"
#include "shell.h"
#include "suction.h"
#include "trajectoryCommander.h"
#include "truthMaker.h"
#include "wallsensor.h"
#include "wheelOdometry.h"

// Activity
#include "ActivityFactory.h"




// プロトタイプ宣言
void halInit();
void startUpInit();
void object_init();


extern "C" void timerInterrupt0();
extern "C" void timerInterrupt1();

void timerInterrupt0() {
    //__builtin_rx_setpsw('I'); 多重割り込み受付

    static uint64_t int_tick_count = 0;
    uint32_t start_usec = hal::getElapsedUsec();

    // 毎回行う処理
    {        
        module::Shell::getInstance().cycle0();
        module::Heater::getInstance().cycle0();
    }
    // スロット0
    if (int_tick_count % 4 == 0) {
        module::ImuDriver::getInstance().cycle0();
        module::WheelOdometry::getInstance().cycle0();
        module::BatteryMonitor::getInstance().cycle0();
        module::PowerTransmission::getInstance().cycle0();
        module::Shell::getInstance().cycle1();

        uint32_t end_usec = hal::getElapsedUsec();        
        hal::setSlot0Time(end_usec - start_usec);
    }
    // スロット1
    if (int_tick_count % 4 == 1) {
        module::LedController::getInstance().cycle0();
        module::Shell::getInstance().cycle1();

        uint32_t end_usec = hal::getElapsedUsec();
        hal::setSlot1Time(end_usec - start_usec);
        
    }
    // スロット2
    if (int_tick_count % 4 == 2) {
        module::WallSensor::getInstance().cycle0();
        module::PseudoDial::getInstance().cycle0();
        module::Navigator::getInstance().cycle2();
        module::Shell::getInstance().cycle1();

        uint32_t end_usec = hal::getElapsedUsec();
        hal::setSlot2Time(end_usec - start_usec);
    }
    // スロット3
    if (int_tick_count % 4 == 3) {        
        module::WallSensor::getInstance().cycle1();        
        module::Shell::getInstance().cycle1();                

        uint32_t end_usec = hal::getElapsedUsec();
        hal::setSlot3Time(end_usec - start_usec);
    }

    
    int_tick_count++;
}

void timerInterrupt1() {

}

int main(void) {
    halInit();
    startUpInit();
    module::LedController::getInstance().turnFcled(1,0,0);
    hal::waitmsec(100);
    module::LedController::getInstance().turnFcled(0,1,0);
    hal::waitmsec(100);
    module::LedController::getInstance().turnFcled(0,0,1);
    hal::waitmsec(100);
    module::LedController::getInstance().turnFcled(0,0,0);
    hal::waitmsec(100);

    while(1) {                
        auto activity = activity::ActivityFactory::cteateModeSelect();
        activity->start();
    }
    return 0;
}


//各ペリフェラルの初期化
void halInit() {
    hal::initClock();
    hal::initGpio();

    hal::initTimer();
    hal::initUart0();
    hal::initUart1();
    hal::initAD();
    hal::initFlashRom();
    hal::initPWM0();
    hal::initPWM1();
    hal::initPWM2();
    hal::initPWM3();
    hal::initPWM4();
    hal::initPWM5();

    hal::initSPI0();
    hal::initSPI1();
    hal::initTimerInterrupt0();
    //hal::initTimerInterrupt1();
    //hal::initWdt();    
}


//起動時の処理
void startUpInit() {
    object_init();
    hal::setPriorityTimerInterrupt0(15);
    hal::startTimerInterrupt0();
}

void object_init() {
    module::BatteryMonitor::getInstance().setDeltaT(0.001f);
    module::ControlMixer::getInstance().setDeltaT(0.001f);
    module::Gamepad::getInstance().setDeltaT(0.001f);
    module::Heater::getInstance().setDeltaT(0.00025f);
    module::ImuDriver::getInstance().setDeltaT(0.001f);
    module::LedController::getInstance().setDeltaT(0.001f);
    module::Navigator::getInstance().setDeltaT(0.001f);
    module::ParameterManager::getInstance();
    module::PositionEstimator::getInstance().setDeltaT(0.001f);
    module::PowerTransmission::getInstance().setDeltaT(0.001f);
    module::PseudoDial::getInstance().setDeltaT(0.001f);
    module::SeManager::getInstance();
    module::Shell::getInstance();
    module::Suction::getInstance().setDeltaT(0.00025f);
    module::WallSensor::getInstance();    
    module::WheelOdometry::getInstance().setDeltaT(0.001f);
}


