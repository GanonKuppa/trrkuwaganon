#include "modeSelectActivity.h"
#include <stdint.h>
#include <string>

// Module
#include "baseActivity.h"
#include "ledController.h"
#include "pseudoDial.h"
#include "seManager.h"
#include "navigator.h"

// Activity
#include "activityFactory.h"

// Msg
#include "msgBroker.h"
#include "dialPositionMsg.h"
#include "gamepadMsg.h"


namespace activity {

    ModeSelectActivity::ModeSelectActivity() :
    _mode(0),
    _mode_num(8)
    { }

    std::string ModeSelectActivity::getModeName()
    {
        std::string mode_name("ModeSelectActivity");
        return mode_name;
    }

    void ModeSelectActivity::onStart() {

        module::PseudoDial& pd = module::PseudoDial::getInstance();
        pd.setEnable(false);
        pd.reset();
        pd.setDivisionNum(_mode_num, 8);
        pd.setEnable(true);

        module::Navigator& nav = module::Navigator::getInstance();
        nav.setNavMode(ENavMode::MODE_SELECT);
        nav.setNavSubMode(ENavSubMode::STANDBY);

        module::LedController::getInstance().turnFcled(1,0,0);
        hal::waitmsec(100);
        module::LedController::getInstance().turnFcled(0,1,0);
        hal::waitmsec(100);
        module::LedController::getInstance().turnFcled(0,0,1);
        hal::waitmsec(100);
        module::LedController::getInstance().turnFcled(0,0,0);
        hal::waitmsec(100);


        _turnFcled(_mode, false);
    }

    void ModeSelectActivity::onFinish() {
        module::PseudoDial& pd = module::PseudoDial::getInstance();
        pd.reset();
        pd.setEnable(false);

        EActivityColor color = _modeNum2Color(_mode);
        PRINTF_ASYNC("color num = %d\n",int(color));

        auto activity = ActivityFactory::create(color);
        activity->start();
    }

    ModeSelectActivity::ELoopStatus ModeSelectActivity::loop() {        
        GamepadMsg gp_msg;
        DialPositionMsg dp_msg;        

        copyMsg(msg_id::GAMEPAD, &gp_msg);
        copyMsg(msg_id::DIAL_POSITION, &dp_msg);
        uint8_t mode_pre = _mode;
        if(gp_msg.connected){
            if(gp_msg.cross_y == 1){
                _mode =  (_mode + _mode_num + 1) % _mode_num;
                hal::waitmsec(100);
                sound::cursor_move();
            }
            
            if(gp_msg.cross_y == -1){
                _mode =  (_mode + _mode_num - 1) % _mode_num;
                hal::waitmsec(100);
                sound::cursor_move();
            }

            if(gp_msg.B > 2000){
                hal::waitmsec(100);
                sound::confirm();
                return ELoopStatus::FINISH;
            }
        }
        else{
            _mode = dp_msg.dial_pos_l;

            if(dp_msg.dial_pos_r  == 4 && dp_msg.same_pos_time_r > 0.1f) {
                return ELoopStatus::FINISH;
            }

        } 

        bool able_goal = false;//m.maze.isExistPath(pm.goal_x, pm.goal_y);
        if(_mode != mode_pre){
            _turnFcled(_mode, able_goal);
        }
        return ELoopStatus::CONTINUE;
    }

    void ModeSelectActivity::_turnFcled(uint8_t mode, bool able_goal) {
        module::LedController& fcled = module::LedController::getInstance();

        if(able_goal) {
            if      (mode == 0) fcled.turnFcled(0, 0, 0);            // BLACK
            else if (mode == 1) fcled.flashFcled(1, 0, 0, 0.4, 0.1); // RED
            else if (mode == 2) fcled.flashFcled(0, 1, 0, 0.4, 0.1); // GREEN
            else if (mode == 3) fcled.flashFcled(1, 1, 0, 0.4, 0.1); // YELLOW
            else if (mode == 4) fcled.flashFcled(0, 0, 1, 0.4, 0.1); // BLUE
            else if (mode == 5) fcled.flashFcled(1, 0, 1, 0.4, 0.1); // MAGENTA
            else if (mode == 6) fcled.flashFcled(0, 1, 1, 0.4, 0.1); // CYAN
            else if (mode == 7) fcled.flashFcled(1, 1, 1, 0.4, 0.1); // WHITE
        } else {
            if      (mode == 0) fcled.turnFcled(0, 0, 0); // BLACK
            else if (mode == 1) fcled.turnFcled(1, 0, 0); // RED
            else if (mode == 2) fcled.turnFcled(0, 1, 0); // GREEN
            else if (mode == 3) fcled.turnFcled(1, 1, 0); // YELLOW
            else if (mode == 4) fcled.turnFcled(0, 0, 1); // BLUE
            else if (mode == 5) fcled.turnFcled(1, 0, 1); // MAGENTA
            else if (mode == 6) fcled.turnFcled(0, 1, 1); // CYAN
            else if (mode == 7) fcled.turnFcled(1, 1, 1); // WHITE
        }
    }


    EActivityColor ModeSelectActivity::_modeNum2Color(uint8_t mode) {
        EActivityColor color;
        color = (EActivityColor)mode;
        return color;
    }


}
