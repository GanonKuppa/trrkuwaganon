#include "fullAutoRunActivity.h"


namespace activity{    


    std::string FullAutoRunActivity::getModeName()
    {
        std::string mode_name = "FullAutoRunActivity";
        return mode_name;
    }

    void FullAutoRunActivity::onStart(){

    }
    
    
    void FullAutoRunActivity::onFinish(){

    }


    FullAutoRunActivity::ELoopStatus FullAutoRunActivity::loop() {

        return ELoopStatus::CONTINUE;
    }
}

