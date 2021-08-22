#pragma once

#include "baseModule.h"

namespace module {
    class PseudoDial: public BaseModule<PseudoDial> {
      public:
        void update0();

        friend class BaseModule<PseudoDial>;
        PseudoDial();
    };

    int usrcmd_pseudoDial(int argc, char **argv);

}
