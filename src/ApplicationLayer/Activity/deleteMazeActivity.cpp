#include "deleteMazeActivity.h"


namespace activity{    


    std::string DeleteMazeActivity::getModeName()
    {
        std::string mode_name = "DeleteMazeActivity";
        return mode_name;
    }

    void DeleteMazeActivity::onStart(){

    }
    
    
    void DeleteMazeActivity::onFinish(){

    }


    DeleteMazeActivity::ELoopStatus DeleteMazeActivity::loop() {

        return ELoopStatus::CONTINUE;
    }
}

