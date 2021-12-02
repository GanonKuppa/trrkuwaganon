#pragma once

#include "intent.h"
#include "hal_timer.h"
#include <utility>
#include <memory>
#include <string>

#include "debugLog.h"
#include "oneshotTask.h"
#include "shell.h"
#include "navigator.h"

namespace activity {

    class BaseActivity {
      public:
        void start(Intent intent) {
            _lower_limit_loop_usec = 500;
            _intent = intent;
            
            PRINTF_ASYNC("\n");
            PRINTF_ASYNC("--- %s start ---\n", getModeName().c_str());

            onStart();
            
            while(1) {
                uint64_t start_usec = hal::getElapsedUsec();
                module::Shell::getInstance().cycleInMainLoop();
                module::Navigator::getInstance().cycleInMainLoop();
                
                scheduler::doTask();
                if(loop() == ELoopStatus::FINISH) break;
                uint64_t end_usec = hal::getElapsedUsec();
                uint64_t elapsed_usec = end_usec - start_usec;
                int64_t wait_usec = _lower_limit_loop_usec - elapsed_usec;
                if(wait_usec > 0) hal::waitusec(wait_usec);
            }

            PRINTF_ASYNC("--- %s end ---\n", getModeName().c_str());
            onFinish();
        }

        void start() {
            Intent intent = Intent();
            start(intent);
        }

        Intent getIntent(){
            return _intent;
        }

        virtual ~BaseActivity() {};

      protected:
        enum class ELoopStatus : uint8_t{
            CONTINUE = 1,
            FINISH = 0
        };

        virtual std::string getModeName() = 0;
        virtual ELoopStatus loop() = 0;
        virtual void onStart() = 0;
        virtual void onFinish() = 0;

        uint32_t _lower_limit_loop_usec;
        Intent _intent;

    };

}
