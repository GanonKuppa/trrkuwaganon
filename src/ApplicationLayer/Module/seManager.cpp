#include "seManager.h"

#ifndef SILS

#else
#include <windows.h>
#endif


namespace sound {
    void cursor_move(){
#ifndef SILS
        
#else
        Beep( 3951, 100 );
#endif
    }

    void confirm(){
#ifndef SILS
        
#else
        Beep( 3520, 60 );
        Beep( 3520, 60 );
#endif
    }

    void on_activity(){

    }

    void end_activity(){

    }

    void startup(){

    }

    void goal(){

    }

    void error(){

    }

    void sensor_calib(){

    }

}