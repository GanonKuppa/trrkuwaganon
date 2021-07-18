
#include <queue>
#include <functional>
#include "hal_timerInterrupt.h"
#include "oneshotTask.h"

namespace scheduler{


static std::queue<std::function<void(void)>> task_que;

    void pushTask(std::function<void(void)> f){
        task_que.emplace(f);
    };


    void doTask(){
        hal::stopTimerInterrupt0();
        bool empty;
        if(!task_que.empty()){                

        	hal::startTimerInterrupt0();
            task_que.front()();
            hal::startTimerInterrupt0();
            task_que.pop();
            hal::startTimerInterrupt0();
        }
        else{
            hal::startTimerInterrupt0();
        }
    };
}
