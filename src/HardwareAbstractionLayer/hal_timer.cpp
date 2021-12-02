#include <stdint.h>
#include "hal_timer.h"

#ifndef SILS
#include <PeripheralDriverLayer/pd_timer.h>
#else
#include <chrono>
#include <unistd.h>
static std::chrono::system_clock::time_point start;

#endif
namespace hal {
    void initTimer() {
#ifndef SILS        
        peripheral_driver::initCMTW0();
        peripheral_driver::initCMTW1();
        peripheral_driver::startCMTW();        
        peripheral_driver::initTPU0();
#else
        start = std::chrono::system_clock::now(); // 計測開始時間
#endif
    }

    void waitClockCount(uint32_t cCount) {
#ifndef SILS
        peripheral_driver::waitClockCount(cCount);
#endif
    }

    void waitnsec(uint32_t nsec) {
#ifndef SILS
        peripheral_driver::waitnsec(nsec);
#else
        nanosleep(nsec);
#endif
    }

    void waitusec(uint32_t usec) {
#ifndef SILS
        peripheral_driver::waitusec(usec);
#else
        usleep(usec);
#endif
    }

    void waitmsec(uint32_t msec) {
#ifndef SILS
        peripheral_driver::waitmsec(msec);
#else
        usleep(msec * 1000);
#endif
    }

    uint64_t getElapsedClockCount() {
#ifndef SILS
        return peripheral_driver::getElapsedClockCount();
#else
        std::chrono::system_clock::time_point end; // 型は auto で可
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
        return (uint64_t)elapsed;
#endif
    }

    uint64_t getElapsedNsec() {
#ifndef SILS
        return peripheral_driver::getElapsedNsec();
#else
        std::chrono::system_clock::time_point end; // 型は auto で可
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
        return (uint64_t)elapsed;
#endif
    }


    uint64_t getElapsedUsec() {
#ifndef SILS
        return peripheral_driver::getElapsedUsec();
#else
        std::chrono::system_clock::time_point end; // 型は auto で可
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
        return (uint32_t)elapsed;
#endif
    }

    uint32_t getElapsedMsec() {
#ifndef SILS
        return peripheral_driver::getElapsedMsec();
#else
        std::chrono::system_clock::time_point end; // 型は auto で可
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count();
        return (uint32_t)elapsed;
#endif
    }

    uint32_t getElapsedSec() {
#ifndef SILS
        return peripheral_driver::getElapsedSec();
#else
        std::chrono::system_clock::time_point end; // 型は auto で可
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::seconds>(end-start).count();
        return (uint32_t)elapsed;
#endif
    }

    float calcElapsedUsec(uint64_t clock_count){
#ifndef SILS
        return peripheral_driver::calcElapsedUsec(clock_count);
#else
        std::chrono::system_clock::time_point end; // 型は auto で可
        end = std::chrono::system_clock::now();  // 計測終了時間
        double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();                
        return elepased - double(clock_count);
#endif
    }

    void hrtStartTimer(){
#ifndef SILS
        return peripheral_driver::hrtStartTimer();
#else

#endif        
    }

    void hrtStopTimer(){
#ifndef SILS
        return peripheral_driver::hrtStopTimer();
#else

#endif
    }

    float hrtGetElapsedUsec(float usec){
#ifndef SILS
        return peripheral_driver::hrtGetElapsedUsec(usec);
#else
        return 0.0f;
#endif
    }

}
