#pragma once

#include "baseModule.h"

// Object
#include "maze.h"

#define FULL_PARAM 1

namespace module {
    class Logger : public BaseModule<Logger> {
      public:
        void update3();

        void printHeadder();
        void print();
        void printPickleMsg();
        void debug();        
        void start();        
        void end();
        void update();

      private:      
        friend class BaseModule<Logger>;
        Logger();

        uint32_t _data_num;
        uint32_t _start_time_ms;
        uint8_t  _skip_mod;
        #if FULL_PARAM        
        const uint32_t _max_data_num = 1500;
        #else
        const uint32_t _max_data_num = 3000;
        #endif

        bool _logging;
    };

    int usrcmd_logger(int argc, char **argv);

}
