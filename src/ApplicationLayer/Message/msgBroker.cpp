#include "msgBroker.h"
#include "baseMsg.h"

#include "batteryInfoMsg.h"
#include "ctrlSetpointMsg.h"
#include "actuatorOutputMsg.h"
#include "imuMsg.h"
#include "wheelOdometryMsg.h"
#include "navStateMsg.h"
#include "wallSensorMsg.h"
#include "vehiclePositionMsg.h"
#include "VehicleAttitudeMsg.h"
#include "groundTruthMsg.h"
#include "gamepadMsg.h"

static BatteryInfoMsg batteryInfoMsg;
static CtrlSetpointMsg ctrlSetpointMsg;
static ActuatorOutputMsg actuatorOutputMsg;
static ImuMsg imuMsg;
static WheelOdometryMsg wheelOdometryMsg;
static NavStateMsg navStateMsg;
static WallSensorMsg wallSensorMsg;
static VehiclePositionMsg vehiclePositionMsg;
static VehicleAttitudeMsg vehicleAttitudeMsg;
static GroundTruthMsg groundTruthMsg;
static GamepadMsg gamepadMsg;



void publishMsg(msg_id msg_id, void* msg){
    if     (msg_id == msg_id::BATTERY_INFO)      {batteryInfoMsg       = *(BatteryInfoMsg*)msg      ;}
    else if(msg_id == msg_id::CTRL_SETPOINT)     {ctrlSetpointMsg      = *(CtrlSetpointMsg*)msg     ;}
    else if(msg_id == msg_id::ACTUATOR_OUTPUT)   {actuatorOutputMsg    = *(ActuatorOutputMsg*)msg   ;}
    else if(msg_id == msg_id::IMU)               {imuMsg               = *(ImuMsg*)msg              ;}
    else if(msg_id == msg_id::WHEEL_ODOMETRY)    {wheelOdometryMsg     = *(WheelOdometryMsg*)msg    ;}
    else if(msg_id == msg_id::NAV_STATE)         {navStateMsg          = *(NavStateMsg*)msg         ;}
    else if(msg_id == msg_id::WALL_SENSOR)       {wallSensorMsg        = *(WallSensorMsg*)msg       ;}
    else if(msg_id == msg_id::VEHICLE_POSITION)  {vehiclePositionMsg   = *(VehiclePositionMsg*)msg  ;}
    else if(msg_id == msg_id::VEHICLE_ATTITUDE)  {vehicleAttitudeMsg   = *(VehicleAttitudeMsg*)msg  ;}
    else if(msg_id == msg_id::GROUND_TRUTH)      {groundTruthMsg       = *(GroundTruthMsg*)msg      ;}
    else if(msg_id == msg_id::GAMEPAD)           {gamepadMsg           = *(GamepadMsg*)msg          ;}  
}

void copyMsg(msg_id msg_id, void *msg){
    if     (msg_id == msg_id::BATTERY_INFO)      {*(BatteryInfoMsg*      ) msg = batteryInfoMsg      ;}
    else if(msg_id == msg_id::CTRL_SETPOINT)     {*(CtrlSetpointMsg*     ) msg = ctrlSetpointMsg     ;}
    else if(msg_id == msg_id::ACTUATOR_OUTPUT)   {*(ActuatorOutputMsg*   ) msg = actuatorOutputMsg   ;}
    else if(msg_id == msg_id::IMU)               {*(ImuMsg*              ) msg = imuMsg              ;}
    else if(msg_id == msg_id::WHEEL_ODOMETRY)    {*(WheelOdometryMsg*    ) msg = wheelOdometryMsg    ;}
    else if(msg_id == msg_id::NAV_STATE)         {*(NavStateMsg*         ) msg = navStateMsg         ;}    
    else if(msg_id == msg_id::WALL_SENSOR)       {*(WallSensorMsg*       ) msg = wallSensorMsg       ;}
    else if(msg_id == msg_id::VEHICLE_POSITION)  {*(VehiclePositionMsg*  ) msg = vehiclePositionMsg  ;}
    else if(msg_id == msg_id::VEHICLE_ATTITUDE)  {*(VehicleAttitudeMsg*  ) msg = vehicleAttitudeMsg  ;}
    else if(msg_id == msg_id::GROUND_TRUTH)      {*(GroundTruthMsg*      ) msg = groundTruthMsg      ;}
    else if(msg_id == msg_id::GAMEPAD)           {*(GamepadMsg*          ) msg = gamepadMsg          ;}    
}
