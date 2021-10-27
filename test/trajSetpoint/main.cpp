#include <stdio.h>
#include "turnPreCalculation.h"
#include "trajectoryFactory.h"

#include "msgBroker.h"
#include "ctrlSetpointMsg.h"

int main() {
    printf("-------- TrajSetpoint test --------\n");
    {
        printf("-- Straight1 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.0f, 1.0f, 0.0f, 10.0f, 10.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.0f, 1.0f, 0.0f, 10.0f, 10.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Straight2 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.0f, 5.0f, 0.0f, 10.0f, 10.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.0f, 5.0f, 0.0f, 10.0f, 10.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Straight3 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.5f, 0.9f, 0.9f, 3.0f, 3.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.9f, 0.9f, 0.5f, 3.0f, 3.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Straight4 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.9f, 0.9f, 0.5f, 3.0f, 3.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.9f, 0.9f, 0.5f, 10.0f, 10.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Straight5 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.5f, 0.7f, 0.6f, 3.0f, 3.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.5f, 0.7f, 0.6f, 3.0f, 3.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Straight6 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.5f, 0.7f, 0.4f, 3.0f, 3.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT, 0.9f, 0.5f, 0.7f, 0.4f, 3.0f, 3.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

   {
        printf("-- Straight7 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT_CENTER_EDGE, 1.5f, 0.0f, 3.5f, 0.8f, 10.0f, 10.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT_CENTER_EDGE, 1.5f, 0.0f, 3.5f, 0.8f, 10.0f, 10.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

{
        printf("-- Straight8 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StraightFactory::create(ETurnType::STRAIGHT_CENTER_EDGE, 1.5f, 0.0f, 3.5f, 0.0f, 10.0f, 10.0f);
        traj->setInitPos(0.0f, 0.0f, 3.14159265f / 2.0f);
        while(!traj->isEnd()){
            traj->publish();
            CtrlSetpointMsg msg;
            copyMsg(msg_id::CTRL_SETPOINT, &msg);
            traj->update();
            printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %s, %s, %s\n",
                t,
                msg.x,
                msg.v_x,
                msg.a_x,
                msg.y,
                msg.v_y,
                msg.a_y,
                msg.v_xy_body,
                msg.a_xy_body,
                msg.yaw,
                msg.yawrate,
                msg.yawacc,
                msg.beta,
                msg.beta_dot,
                trajType2Str(msg.traj_type).c_str(),
                turnType2Str(msg.turn_type).c_str(),
                turnDir2Str(msg.turn_dir).c_str()
            );

            t += delta_t;
        }
        traj = StraightFactory::create(ETurnType::STRAIGHT_CENTER_EDGE, 1.5f, 0.0f, 3.5f, 0.0f, 10.0f, 10.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }


    return 0;
}
