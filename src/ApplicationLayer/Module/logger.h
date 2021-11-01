#pragma once

#include "baseModule.h"

// Object
#include "maze.h"

#define FULL_PARAM 0

namespace module {
    class Logger : public BaseModule<Logger> {
      public:
        void update3();

        void printHeadder();
        void print();
        void debug();
        void start(uint8_t skip_mod = 0);
        void end();
        void update();

      private:      
        friend class BaseModule<Logger>;
        Logger();

        uint32_t _data_num;
        uint32_t _start_time_ms;
        uint8_t  _skip_mod;
        #if FULL_PARAM        
        const uint32_t _max_data_num = 1200;
        #else
        const uint32_t _max_data_num = 3000;
        #endif

        bool _logging;
    };

    int usrcmd_logger(int argc, char **argv);

}