#pragma once

#include "baseModule.h"
#include "led.h"
#include <memory>
#include <stdint.h>

namespace module {
    class LedController : public BaseModule<LedController> {
      public:
        void update1();
        void turnFcled(bool r, bool g, bool b);
        void flashFcled(bool r, bool g, bool b, float on_time, float off_time);
        void oneshotFcled(bool r, bool g, bool b, float on_time, float off_time);
        uint8_t getFcledState();
        void setDeltaT(float delta_t);
        void debug();
      private:
        std::unique_ptr<Led> _led_r;
        std::unique_ptr<Led> _led_g;
        std::unique_ptr<Led> _led_b;
        bool _oneshot_engaged;
        float _oneshot_time;

        friend class BaseModule<LedController>;
        LedController();
    };

    int usrcmd_ledController(int argc, char **argv);

}
