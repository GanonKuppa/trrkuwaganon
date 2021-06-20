#pragma once 

#include "baseModule.h"
#include <stdarg.h>

namespace module {

    class Communication : public BaseModule<Communication> {
      public:
        void update();
        int printfAsync(const char* fmt, ...);
        int printfSync(const char* fmt, ...);
      private:
        friend class BaseModule<Communication>;
        Communication();
    };

}


