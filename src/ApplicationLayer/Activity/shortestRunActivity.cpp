#include "shortestRunActivity.h"


namespace activity{    


    std::string ShortestRunActivity::getModeName()
    {
        std::string mode_name = "ShortestRunActivity";
        return mode_name;
    }

    void ShortestRunActivity::onStart(){

    }
    
    
    void ShortestRunActivity::onFinish(){

    }


    ShortestRunActivity::ELoopStatus ShortestRunActivity::loop() {

        return ELoopStatus::CONTINUE;
    }
}

