#pragma once

#include "hal_timer.h"
#include "stdint.h"
#include "debugLog.h"
#include <string>

namespace module {
    template <class T>
    class BaseModule {
      public:
        virtual void update0(){};
        virtual void update1(){};
        virtual void update2(){};
        virtual void update3(){};
        virtual void setDeltaT(float delta_t) {_delta_t = delta_t;}
        
        void cycle0(){
            uint32_t start_time = hal::getElapsedUsec();
            update0();
            _cycle_time_us[0] = hal::getElapsedUsec() - start_time;
        }

        void cycle1(){
            uint32_t start_time = hal::getElapsedUsec();
            update1();
            _cycle_time_us[1] = hal::getElapsedUsec() - start_time;  
        }

        void cycle2(){
            uint32_t start_time = hal::getElapsedUsec();
            update2();
            _cycle_time_us[2] = hal::getElapsedUsec() - start_time;
        }

        void cycle3(){
            uint32_t start_time = hal::getElapsedUsec();
            update3();
            _cycle_time_us[3] = hal::getElapsedUsec() - start_time;
        }

        static T& getInstance() {
            static T _instance;
            return _instance;
        }

        void printCycleTime(){
            PRINTF_ASYNC("    %s\n",getModuleName().c_str());
            PRINTF_ASYNC("      %3d[us], %3d[us], %3d[us], %3d[us]\n\n", _cycle_time_us[0], _cycle_time_us[1], _cycle_time_us[2], _cycle_time_us[3]);
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
        uint32_t _cycle_time_us[4];
        BaseModule(): _delta_t(0.001f), 
                      _cycle_time_us{0, 0, 0, 0},
                      _module_name("BaseModule")
        { }

      private:
        BaseModule(const BaseModule&) = delete;
        BaseModule& operator=(const BaseModule&) = delete;
        BaseModule(BaseModule&&) = delete;
        BaseModule& operator=(BaseModule&&) = delete;

    };
}
