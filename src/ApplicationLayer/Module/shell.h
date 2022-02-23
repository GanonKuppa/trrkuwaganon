#pragma once

#include "baseModule.h"
#include "ntshell.h"

namespace module {

    class Shell : public BaseModule<Shell> {
      public:
        void updateEvery();
        void update0();
        void update1();
        void update2();
        void update3();
        void updateInMainLoop();
        ntshell_t* getNtsPtr(){
            return &nts;
        }
      private:
        friend class BaseModule<Shell>;

        ntshell_t nts;
        Shell();
    };

    int usrcmd_shell(int argc, char **argv);

}
