#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include "hal_timer.h"

#include "sendData2Sim.h"

int main() {    
    sim::initSimConnection();
    
    float x, y, t = 0.0f;
    while(1){
        sim::setRobotPos(x,y,90.0f,0);
        hal::waitmsec(10);
        
        x = sinf(t);
        y = cosf(t);
        t += 0.01f;
    }

    return 0;
}
