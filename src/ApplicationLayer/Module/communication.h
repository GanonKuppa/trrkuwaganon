#pragma once 

#include "baseModule.h"


namespace module {

    class Communication : public BaseModule<Communication> {
      public:
        void update();
      private:
        friend class BaseModule<Communication>;
        Communication();
    };

}


