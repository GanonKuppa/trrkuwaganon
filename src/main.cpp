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
#include "logger.h"
#include "navigator.h"
#include "parameterManager.h"
#include "positionEstimator.h"
#include "powerTransmission.h"
#include "pseudoDial.h"
#include "runAnalizer.h"
#include "seManager.h"
#include "shell.h"
#include "suction.h"
#include "trajectoryCommander.h"
#include "trajectoryInitializer.h"
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
    hal::enableMultipleinterrupt();

    static uint64_t int_tick_count = 0;
    //uint64_t start_clock_count = hal::getElapsedClockCount();
    hal::hrtStartTimer();

    // 毎回行う処理
    {        
        module::Shell::getInstance().cycleEvery();
        module::Heater::getInstance().cycleEvery();
        module::Suction::getInstance().cycleEvery();
    }
    // スロット0
    if (int_tick_count % 4 == 0) {        
        module::TrajectoryCommander::getInstance().cycle0();
        module::ImuDriver::getInstance().cycle0();
        module::WheelOdometry::getInstance().cycle0();            
        module::PositionEstimator::getInstance().cycle0();
        module::ControlMixer::getInstance().cycle0();
        module::BatteryMonitor::getInstance().cycle0();
        module::PowerTransmission::getInstance().cycle0();
        module::Shell::getInstance().cycle0();        
        module::Navigator::getInstance().cycle0();
        module::WallSensor::getInstance().cycle0();
        module::RunAnalizer::getInstance().cycle0();
        hal::setSlot0Time(hal::hrtGetElapsedUsec());
    }
    // スロット1
    if (int_tick_count % 4 == 1) {        
        module::LedController::getInstance().cycle1();
        module::ImuDriver::getInstance().cycle1();
        module::Shell::getInstance().cycle1();
        hal::setSlot1Time(hal::hrtGetElapsedUsec());
    }
    // スロット2
    if (int_tick_count % 4 == 2) {        
        module::PseudoDial::getInstance().cycle2();        
        module::ImuDriver::getInstance().cycle2();
        module::Shell::getInstance().cycle2();        
        hal::setSlot2Time(hal::hrtGetElapsedUsec());
    }
    // スロット3
    if (int_tick_count % 4 == 3) {                
        module::Logger::getInstance().cycle3();
        module::ImuDriver::getInstance().cycle3();
        module::Shell::getInstance().cycle3();        
        hal::setSlot3Time(hal::hrtGetElapsedUsec());
    }

    //hal::hrtStopTimer();
    int_tick_count++;
}

void timerInterrupt1() {
    module::WallSensor::getInstance().emitLedTask();
}

int main(void) {
    halInit();
    startUpInit();
    module::LedController::getInstance().turnFcled(1,1,1);
    hal::waitmsec(250);
    module::LedController::getInstance().turnFcled(0,0,0);
    hal::waitmsec(50);
    module::LedController::getInstance().turnFcled(1,1,1);
    hal::waitmsec(50);
    module::LedController::getInstance().turnFcled(0,0,0);
    hal::waitmsec(50);

    while(1) {                
        auto activity = activity::ActivityFactory::createModeSelect();
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
    hal::initPickle();
    hal::initTimerInterrupt1();
    //hal::initWdt();    
}


//起動時の処理
void startUpInit() {
    object_init();
    hal::setPriorityTimerInterrupt0(14);
    hal::setPriorityTimerInterrupt1(15);
    hal::setInterruptPeriodTimerInterrupt1(80);
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
    module::RunAnalizer::getInstance();
    module::SeManager::getInstance();
    module::Shell::getInstance();
    module::Suction::getInstance().setDeltaT(0.00025f);
    module::TrajectoryCommander::getInstance().setDeltaT(0.001f);
    module::TrajectoryInitializer::getInstance();
    module::WallSensor::getInstance();    
    module::WheelOdometry::getInstance().setDeltaT(0.001f);    
}


