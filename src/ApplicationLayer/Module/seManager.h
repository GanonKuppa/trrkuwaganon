#include "baseModule.h"
#include <memory>
#include <stdint.h>


namespace sound {
    void cursor_move();
    void confirm();
    void on_activity();
    void end_activity();
    void startup();
    void goal();
    void error();
    void sensor_calib();
}


/*
namespace module {





    class SeManager : public BaseModule<SeManager> {
      public:
        void update();

        enum

      private:
        friend class BaseModule<SeManager>;
        SeManager();
    };
}
*/