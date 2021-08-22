#pragma once

#include "baseModule.h"
#include <stdint.h>

namespace module {
    class ControlMixer : public BaseModule<ControlMixer> {

      private:
        friend class BaseModule<ControlMixer>;
        
        ControlMixer();
        void _publish();
    };

    int usrcmd_controlMixer(int argc, char **argv);
}
