#include "pseudoDial.h"

// Lib
#include "debugLog.h"
#include "ntlibc.h"

// Module
#include "parameterManager.h"

// Msg
#include "msgBroker.h"
#include "dialPositionMsg.h"
#include "navStateMsg.h"
#include "actuatorOutputMsg.h"

namespace module{
    PseudoDial::PseudoDial(){
        setModuleName("PseudoDial");
        ParameterManager& pm = ParameterManager::getInstance();
        reset();
        
        dial_l.setPiGain(pm.dial_p, pm.dial_i, pm.dial_i_limit, pm.dial_limit);
        dial_r.setPiGain(pm.dial_p, pm.dial_i, pm.dial_i_limit, pm.dial_limit);
    }

    void PseudoDial::update0(){
        NavStateMsg ns_msg;
        copyMsg(msg_id::NAV_STATE, &ns_msg);        
                
        if((!ns_msg.armed && ns_msg.mode == ENavMode::MODE_SELECT)){
            ParameterManager& pm = ParameterManager::getInstance();
            dial_l.setPiGain(pm.dial_p, pm.dial_i, pm.dial_i_limit, pm.dial_limit);
            dial_r.setPiGain(pm.dial_p, pm.dial_i, pm.dial_i_limit, pm.dial_limit);
            setEnable(true);
            dial_l.update();
            dial_r.update();
            _publish_dial_position();
            _publish_actuator_output();

        }
        else {
            dial_l.setEnable(false);
            dial_r.setEnable(false);
        }                
    }

    void PseudoDial::debug(){
        PRINTF_ASYNC("  ----------- dial_l -----------\n");
        dial_l.debug();
        PRINTF_ASYNC("  ----------- dial_r -----------\n");
        dial_r.debug();
    }

    void PseudoDial::_publish_dial_position(){
        DialPositionMsg msg;
        msg.dial_pos_l = dial_l.getDialPosition();
        msg.dial_pos_r = dial_r.getDialPosition();
        msg.dial_div_num_l = dial_l.getDivisionNum();
        msg.dial_div_num_r = dial_r.getDivisionNum();
        publishMsg(msg_id::DIAL_POSITION, &msg);        
    }

    void PseudoDial::_publish_actuator_output(){
        ActuatorOutputMsg msg;
        msg.duty_l = dial_l.getDuty();
        msg.duty_r = dial_r.getDuty();        
        msg.ctrl_mode = ECtrlMode::PSEUDO_DIAL;
        publishMsg(msg_id::ACTUATOR_OUTPUT, &msg);
    }

    void PseudoDial::setEnable(bool enable){
        dial_l.setEnable(enable);
        dial_r.setEnable(enable);
    }

    void PseudoDial::setDivisionNum(uint8_t l_num, uint8_t r_num){
        dial_l.setDivisionNum(l_num);
        dial_r.setDivisionNum(r_num);
    }

    void PseudoDial::reset(){
        dial_l.reset();
        dial_r.reset();                
    }    

    int usrcmd_pseudoDial(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            PseudoDial::getInstance().debug();
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;
    };




}
