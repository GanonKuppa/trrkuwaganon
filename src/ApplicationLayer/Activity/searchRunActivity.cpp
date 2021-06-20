#include "searchRunActivity.h"

namespace activity{    


    std::string SearchRunActivity::getModeName()
    {
        std::string mode_name = "SearchRunActivity";
        return mode_name;
    }

    void SearchRunActivity::onStart(){

    }
    
    
    void SearchRunActivity::onFinish(){

    }


    SearchRunActivity::ELoopStatus SearchRunActivity::loop() {

        return ELoopStatus::CONTINUE;
    }
}

