#pragma once

#include "intent.h"
#include "hal_timer.h"
#include <utility>
#include <memory>
#include <string>

#include "debugLog.h"
#include "oneshotTask.h"
#include "shell.h"

namespace activity {

    class BaseActivity {
      public:
        void start(Intent intent) {
            _lower_limit_loop_msec = 1;
            _intent = intent;
            
            PRINTF_ASYNC("\n");
            PRINTF_ASYNC("--- %s start ---\n", getModeName().c_str());

            onStart();
            
            while(1) {
                uint32_t start_msec = hal::getElapsedMsec();
                module::Shell::getInstance().cycleInMainLoop();
                scheduler::doTask();
                if(loop() == ELoopStatus::FINISH) break;
                uint32_t end_msec = hal::getElapsedMsec();
                uint32_t elapsed_msec = end_msec - start_msec;
                int32_t wait_msec = _lower_limit_loop_msec - elapsed_msec;
                if(wait_msec > 0) hal::waitmsec(wait_msec);
            };

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

        uint32_t _lower_limit_loop_msec;
        Intent _intent;

    };

}
