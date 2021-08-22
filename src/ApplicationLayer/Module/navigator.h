#pragma once

#include "baseModule.h"

namespace module {
    class Navigator : public BaseModule<Navigator> {
        
      private:
        friend class BaseModule<Navigator>;
        Navigator();
    };

    int usrcmd_navigator(int argc, char **argv);

}
