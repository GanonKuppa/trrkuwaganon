#include <iostream>
#include <Eigen>
#include <thread>

#include <stdint.h>
#include "sendData2Sim.h"
#include <chrono>
#include <iostream>
#include <memory>
#include <time.h>
#include <stdint.h>

#include <XInput.h>
#include <windows.h>



//#pragma comment (lib, "xinput.lib")

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
#include "ledController.h"
#include "gamepad.h"
#include "communication.h"

// Activity
#include "ActivityFactory.h"

// プロトタイプ宣言
void halInit();
void startUpInit();
void object_init();


uint16_t time_count = 0;
float delta_t = 0.00025;
float x = 0.0;
float y = 0.0;
float ang = 0.0;
float v = 0.0;

void timeInterrupt0() {
    static uint64_t count = 0;
    module::LedController::getInstance().update();

    if(count % 4 == 0) module::Gamepad::getInstance().update();

    time_count ++;
    x += delta_t * 0.1;
    y += delta_t * 0.1;
    ang += delta_t * 10;
    count++;
}


void worker1() {
    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    // ループを25msec / delta_t = 100回進める
    for(int i=0; i<100; i++) {
        timeInterrupt0();
    }

    while(1) {
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        if(elapsed > 25 * 1000) {
            //std::cout << elapsed << std::endl;
            break; //25msec経過していたら抜ける
        }
    }

    worker1();
}

void worker2() {
    std::chrono::system_clock::time_point  start, end; // 型は auto で可
    start = std::chrono::system_clock::now(); // 計測開始時間

    uint8_t rgb = module::LedController::getInstance().getFcledState();
    sim::setRobotColor( 255 * (rgb & 0x1), 255 * ((rgb & 0x02) >> 1), 255 * ((rgb & 0x04) >> 2));

    while(1) {
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        if(elapsed > 16 * 1000) {
            //std::cout << elapsed << std::endl;
            break;
        }
    }
    worker2();
}


int main() {
    sim::initSimConnection();
    ////////////////////////////////////////////////////
    halInit();
    startUpInit();
    object_init();
    //module::LedController::getInstance().flashFcled(1,0,1, 1.0, 0.5);
    std::thread t1(worker1);
    std::thread t2(worker2);

    while(1) {
        auto activity = activity::ActivityFactory::cteateModeSelect();
        activity->start();
    }

    sim::finalizeSimConnection();
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
    hal::initDA();
    hal::initFlashRom();
    hal::initPhaseCounting0();
    hal::initPhaseCounting1();
    hal::initPWM0();
    hal::initPWM1();
    hal::initPWM2();
    hal::initPWM3();
    hal::initSPI0();
    hal::initTimerInterrupt0();
    hal::initTimerInterrupt1();
    hal::initWdt();

}


//起動時の処理
void startUpInit() {
    hal::startTimerInterrupt0();
}

void object_init() {
    module::LedController::getInstance().setDeltaT(0.00025);
    module::Gamepad::getInstance();
    module::Communication::getInstance();
}
