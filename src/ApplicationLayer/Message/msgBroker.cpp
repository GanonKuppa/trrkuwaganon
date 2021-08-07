#include "msgBroker.h"
#include "baseMsg.h"

#include "batteryVoltageMsg.h"
#include "ctrlSetpointMsg.h"
#include "gamepadMsg.h"
#include "positionEstimatorMsg.h"
#include "wallSensorMsg.h"
#include "wheelOdometryMsg.h"
#include "imuMsg.h"
#include "groundTruth.h"

static BatteryInfoMsg batteryInfoMsg;
static CtrlSetpointMsg ctrlSetpointMsg;
static GamepadMsg gamepadMsg;
static ImuMsg imuMsg;
static PositionEstimatorMsg positionEstimatorMsg;
static WallSensorMsg wallSensorMsg;
static WheelOdometryMsg wheelOdometryMsg;
static GroundTruthMsg groundTruthMsg;



void publishMsg(msg_id msg_id, void* msg){

    if     (msg_id == msg_id::BATTERY_INFO)      {batteryInfoMsg       = *(BatteryInfoMsg*)msg   ;}
    else if(msg_id == msg_id::CTRL_SETPOINT)     {ctrlSetpointMsg      = *(CtrlSetpointMsg*)msg     ;}
    else if(msg_id == msg_id::GAMEPAD)           {gamepadMsg           = *(GamepadMsg*)msg          ;}    
    else if(msg_id == msg_id::IMU)               {imuMsg               = *(ImuMsg*)msg              ;}
    else if(msg_id == msg_id::POSITION_ESTIMATOR){positionEstimatorMsg = *(PositionEstimatorMsg*)msg;}
    else if(msg_id == msg_id::WALL_SENSOR)       {wallSensorMsg        = *(WallSensorMsg*)msg       ;}
    else if(msg_id == msg_id::WHEEL_ODOMETRY)    {wheelOdometryMsg     = *(WheelOdometryMsg*)msg    ;}
    else if(msg_id == msg_id::GROUND_TRUTH)      {groundTruthMsg        = *(GroundTruthMsg*)msg     ;}

}

void copyMsg(msg_id msg_id, void *msg){
    if     (msg_id == msg_id::BATTERY_INFO)      {*(BatteryInfoMsg*      ) msg = batteryInfoMsg      ;}
    else if(msg_id == msg_id::CTRL_SETPOINT)     {*(CtrlSetpointMsg*     ) msg = ctrlSetpointMsg     ;}
    else if(msg_id == msg_id::GAMEPAD)           {*(GamepadMsg*          ) msg = gamepadMsg          ;}    
    else if(msg_id == msg_id::IMU)               {*(ImuMsg*              ) msg = imuMsg              ;}
    else if(msg_id == msg_id::POSITION_ESTIMATOR){*(PositionEstimatorMsg*) msg = positionEstimatorMsg;}
    else if(msg_id == msg_id::WALL_SENSOR)       {*(WallSensorMsg*       ) msg = wallSensorMsg       ;}
    else if(msg_id == msg_id::WHEEL_ODOMETRY)    {*(WheelOdometryMsg*    ) msg = wheelOdometryMsg    ;}
    else if(msg_id == msg_id::GROUND_TRUTH)      {*(GroundTruthMsg*      ) msg = groundTruthMsg      ;}
}
