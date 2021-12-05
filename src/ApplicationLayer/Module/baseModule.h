#pragma once

#include "hal_timer.h"
#include "stdint.h"
#include "debugLog.h"
#include <string>

namespace module {
    template <class T>
    class BaseModule {
      public:
        virtual void updateEvery(){};
        virtual void update0(){};
        virtual void update1(){};
        virtual void update2(){};
        virtual void update3(){};
        virtual void updateInMainLoop(){};

        virtual void setDeltaT(float delta_t) {_delta_t = delta_t;}
        
        void cycleEvery(){
            float start_usec = hal::hrtGetElapsedUsec();
            updateEvery();
            _cycle_time_every_us = hal::hrtGetElapsedUsec(start_usec);
        }

        void cycle0(){
            float start_usec = hal::hrtGetElapsedUsec();
            update0();
            _cycle_time_us[0] = hal::hrtGetElapsedUsec(start_usec);
        }

        void cycle1(){
            float start_usec = hal::hrtGetElapsedUsec();
            update1();
            _cycle_time_us[1] = hal::hrtGetElapsedUsec(start_usec);
        }

        void cycle2(){
            float start_usec = hal::hrtGetElapsedUsec();
            update2();
            _cycle_time_us[2] = hal::hrtGetElapsedUsec(start_usec);
        }

        void cycle3(){
            float start_usec = hal::hrtGetElapsedUsec();
            update3();
            _cycle_time_us[3] = hal::hrtGetElapsedUsec(start_usec);
        }

        void cycleInMainLoop(){
            uint64_t start_clock_count = hal::getElapsedClockCount();
            updateInMainLoop();
            _cycle_time_in_main_loop_us = hal::calcElapsedUsec(start_clock_count);
        }

        static T& getInstance() {
            static T _instance;
            return _instance;
        }

        virtual void printCycleTime(){
            PRINTF_ASYNC("    %s\n",getModuleName().c_str());
            PRINTF_ASYNC("      %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us], %6.2f[us]\n\n", _cycle_time_us[0], _cycle_time_us[1], _cycle_time_us[2], _cycle_time_us[3], _cycle_time_every_us, _cycle_time_in_main_loop_us);
        }

        std::string getModuleName(){
            return _module_name;
        }

        void setModuleName(std::string name_str){
            _module_name = name_str;
        }

      protected:
        float _delta_t;
        std::string _module_name;
        float _cycle_time_every_us;
        float _cycle_time_us[4];
        float _cycle_time_in_main_loop_us;
        BaseModule(): _delta_t(0.001f), 
                      _module_name("BaseModule"),
                      _cycle_time_every_us(0.0f),
                      _cycle_time_us{0.0f, 0.0f, 0.0f, 0.0f},
                      _cycle_time_in_main_loop_us(0.0f)
        { }

      private:
        BaseModule(const BaseModule&) = delete;
        BaseModule& operator=(const BaseModule&) = delete;
        BaseModule(BaseModule&&) = delete;
        BaseModule& operator=(BaseModule&&) = delete;

    };
}
