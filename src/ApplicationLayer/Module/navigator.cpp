#include "navigator.h"

// Msg
#include "msgBroker.h"
#include "navStateMsg.h"


namespace module{

    Navigator::Navigator(){
        setModuleName("Navigator");
    }

    void Navigator::update2(){
        _publish();        
    }


    void Navigator::_publish(){
        NavStateMsg msg;
        msg.armed = _armed;
        msg.mode = _mode;
        msg.sub_mode = _sub_mode;
        msg.x_cur = _x_cur;
        msg.y_cur = _y_cur;
        msg.x_next = _x_next;
        msg.y_next = _y_next;
        msg.azimuth = _azimuth;
        msg.is_failsafe = _is_failsafe;
        publishMsg(msg_id::ACTUATOR_OUTPUT, &msg);

    }

    int usrcmd_navigator(int argc, char **argv){
        return 0;
    }

}

