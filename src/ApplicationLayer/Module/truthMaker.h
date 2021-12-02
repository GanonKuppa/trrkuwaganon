#pragma once

#include "baseModule.h"

namespace module {

    class TruthMaker : public BaseModule<TruthMaker> {
      public:
        void update0();
      private:
        friend class BaseModule<TruthMaker>;
        TruthMaker();
   };

   int usrcmd_truthMaker(int argc, char **argv);
}
