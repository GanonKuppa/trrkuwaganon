#include <stdio.h>
#include "turnPreCalculation.h"
#include "trajectoryFactory.h"

#include "msgBroker.h"
#include "ctrlSetpointMsg.h"

#include "trajectoryInitializer.h"
#include "parameterManager.h"

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

    {
        printf("-- SpinTurn1 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = SpinTurnFactory::create(3.14159265f, 2000.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = SpinTurnFactory::create(3.14159265f, 2000.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }


    {
        printf("-- SpinTurn2 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = SpinTurnFactory::create(3.14159265f, 500.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = SpinTurnFactory::create(3.14159265f, 500.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- SpinTurn3 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = SpinTurnFactory::create(-3.14159265f, 2000.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = SpinTurnFactory::create(-3.14159265f, 2000.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }


    {
        printf("-- SpinTurn4 --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = SpinTurnFactory::create(-3.14159265f, 500.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = SpinTurnFactory::create(-3.14159265f, 500.0f * 3.14159265f / 180.0f, 5000.0f * 3.14159265f / 180.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Stop --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        auto traj = StopFactory::create(1.0f);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = StopFactory::create(1.0f);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    module::ParameterManager& pm = module::ParameterManager::getInstance();
    pm.v_search_run = 0.3f;
    pm.a_search_run = 3.0f;
    pm.cp_coef = 100.0f;
    
    pm.shortest_0_v = 0.5f;
    pm.shortest_0_v_d= 0.5f;
    pm.shortest_0_v_90= 0.5f;
    pm.shortest_0_v_l90= 0.5f;
    pm.shortest_0_v_180= 0.5f;
    pm.shortest_0_v_d90= 0.5f;
    pm.shortest_0_v_45= 0.5f;
    pm.shortest_0_v_135= 0.5f;
    pm.shortest_0_a = 1.0f;
    pm.shortest_0_a_diag = 1.0f;

    pm.shortest_1_v = 0.5f;
    pm.shortest_1_v_d= 0.5f;
    pm.shortest_1_v_90= 0.5f;
    pm.shortest_1_v_l90= 0.5f;
    pm.shortest_1_v_180= 0.5f;
    pm.shortest_1_v_d90= 0.5f;
    pm.shortest_1_v_45= 0.5f;
    pm.shortest_1_v_135= 0.5f;
    pm.shortest_1_a = 1.0f;
    pm.shortest_1_a_diag = 1.0f;

    pm.shortest_2_v = 0.5f;
    pm.shortest_2_v_d= 0.5f;
    pm.shortest_2_v_90= 0.5f;
    pm.shortest_2_v_l90= 0.5f;
    pm.shortest_2_v_180= 0.5f;
    pm.shortest_2_v_d90= 0.5f;
    pm.shortest_2_v_45= 0.5f;
    pm.shortest_2_v_135= 0.5f;
    pm.shortest_2_a = 1.0f;
    pm.shortest_2_a_diag = 1.0f;

    pm.shortest_3_v = 0.5f;
    pm.shortest_3_v_d= 0.5f;
    pm.shortest_3_v_90= 0.5f;
    pm.shortest_3_v_l90= 0.5f;
    pm.shortest_3_v_180= 0.5f;
    pm.shortest_3_v_d90= 0.5f;
    pm.shortest_3_v_45= 0.5f;
    pm.shortest_3_v_135= 0.5f;
    pm.shortest_3_a = 1.0f;
    pm.shortest_3_a_diag = 1.0f;

    pm.shortest_4_v = 0.5f;
    pm.shortest_4_v_d= 0.5f;
    pm.shortest_4_v_90= 0.5f;
    pm.shortest_4_v_l90= 0.5f;
    pm.shortest_4_v_180= 0.5f;
    pm.shortest_4_v_d90= 0.5f;
    pm.shortest_4_v_45= 0.5f;
    pm.shortest_4_v_135= 0.5f;
    pm.shortest_4_a = 1.0f;
    pm.shortest_4_a_diag = 1.0f;


    module::TrajectoryInitializer::getInstance();
    
    {
        printf("-- Curve180_CW --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        ETurnParamSet param_set = ETurnParamSet::SAFE1;
        ETurnType turn_type = ETurnType::TURN_180;
        ETurnDir turn_dir = ETurnDir::CW;

        auto traj = CurveFactory::create(param_set, turn_type, turn_dir);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = CurveFactory::create(param_set, turn_type, turn_dir);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }

    {
        printf("-- Curve180_CCW --\n");
        float delta_t = 0.001f;
        float t = 0.0f;
        ETurnParamSet param_set = ETurnParamSet::SAFE1;
        ETurnType turn_type = ETurnType::TURN_180;
        ETurnDir turn_dir = ETurnDir::CCW;

        auto traj = CurveFactory::create(param_set, turn_type, turn_dir);
        traj->setInitPos(0.0f, 0.0f, 0.0f);
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
        traj = CurveFactory::create(param_set, turn_type, turn_dir);
        printf("%f, %f, %f, %f\n",traj->getEndX(), traj->getEndY(), traj->getEndYaw(), traj->getNecessaryTime());
    }



    return 0;
}
