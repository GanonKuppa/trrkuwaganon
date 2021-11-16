#pragma once

#include "baseModule.h"
#include "dial.h"

#include "msgBroker.h"
#include "wheelOdometryMsg.h"

class DialL : public Dial{
  private:
    float getAngle() {
        WheelOdometryMsg wo_msg;
        copyMsg(msg_id::WHEEL_ODOMETRY, &wo_msg);
        return wo_msg.ang_l;
    }


    float getVelocity() {
        WheelOdometryMsg wo_msg;
        copyMsg(msg_id::WHEEL_ODOMETRY, &wo_msg);
        return wo_msg.v_l;
    }
};

class DialR : public Dial{
  private:
    float getAngle() {
        WheelOdometryMsg wo_msg;
        copyMsg(msg_id::WHEEL_ODOMETRY, &wo_msg);
        return wo_msg.ang_r;
    }


    float getVelocity() {
        WheelOdometryMsg wo_msg;
        copyMsg(msg_id::WHEEL_ODOMETRY, &wo_msg);
        return wo_msg.v_r;
    }
};


namespace module {
    class PseudoDial: public BaseModule<PseudoDial> {
      public:
        void update2();
        void debug();
        void setDivisionNum(uint8_t l_rum, uint8_t r_num);
        void setEnable(bool enable);
        void reset();
      private:
        friend class BaseModule<PseudoDial>;
        PseudoDial();

        DialL dial_l;
        DialR dial_r;
        void _publish_dial_position();
        void _publish_actuator_output();
        
    };

    int usrcmd_pseudoDial(int argc, char **argv);

}
