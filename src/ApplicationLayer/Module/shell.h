#pragma once

#include "baseModule.h"
#include "ntshell.h"

namespace module {

    class Shell : public BaseModule<Shell> {
      public:
        void update0();
      private:
        friend class BaseModule<Shell>;

        ntshell_t nts;

        Shell();
        static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
        static int user_callback(const char *text, void *extobj);
        static int serial_read_1byte(char *buf, int cnt, void *extobj);
        static int serial_write(const char *buf, int cnt, void *extobj);
    };

}
