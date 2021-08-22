#include "msgBroker.h"
#include "baseMsg.h"

#include "actuatorOutputMsg.h"
#include "batteryInfoMsg.h"
#include "ctrlSetpointMsg.h"
#include "dialPositionMsg.h"
#include "gamepadMsg.h"
#include "groundTruthMsg.h"
#include "imuMsg.h"
#include "navStateMsg.h"
#include "VehicleAttitudeMsg.h"
#include "vehiclePositionMsg.h"
#include "wallSensorMsg.h"
#include "wheelOdometryMsg.h"

static ActuatorOutputMsg actuatorOutputMsg;
static BatteryInfoMsg batteryInfoMsg;
static CtrlSetpointMsg ctrlSetpointMsg;
static DialPositionMsg dialPositionMsg;
static GamepadMsg gamepadMsg;
static GroundTruthMsg groundTruthMsg;
static ImuMsg imuMsg;
static NavStateMsg navStateMsg;
static VehicleAttitudeMsg vehicleAttitudeMsg;
static VehiclePositionMsg vehiclePositionMsg;
static WallSensorMsg wallSensorMsg;
static WheelOdometryMsg wheelOdometryMsg;


void publishMsg(msg_id msg_id, void* msg){
    if     (msg_id == msg_id::ACTUATOR_OUTPUT)   {actuatorOutputMsg    = *(ActuatorOutputMsg*)msg   ;}
    else if(msg_id == msg_id::BATTERY_INFO)      {batteryInfoMsg       = *(BatteryInfoMsg*)msg      ;}
    else if(msg_id == msg_id::CTRL_SETPOINT)     {ctrlSetpointMsg      = *(CtrlSetpointMsg*)msg     ;}
    else if(msg_id == msg_id::DIAL_POSITION)     {dialPositionMsg      = *(DialPositionMsg*)msg     ;}
    else if(msg_id == msg_id::GAMEPAD)           {gamepadMsg           = *(GamepadMsg*)msg          ;}
    else if(msg_id == msg_id::GROUND_TRUTH)      {groundTruthMsg       = *(GroundTruthMsg*)msg      ;}
    else if(msg_id == msg_id::IMU)               {imuMsg               = *(ImuMsg*)msg              ;}
    else if(msg_id == msg_id::NAV_STATE)         {navStateMsg          = *(NavStateMsg*)msg         ;}
    else if(msg_id == msg_id::VEHICLE_ATTITUDE)  {vehicleAttitudeMsg   = *(VehicleAttitudeMsg*)msg  ;}
    else if(msg_id == msg_id::VEHICLE_POSITION)  {vehiclePositionMsg   = *(VehiclePositionMsg*)msg  ;}
    else if(msg_id == msg_id::WALL_SENSOR)       {wallSensorMsg        = *(WallSensorMsg*)msg       ;}
    else if(msg_id == msg_id::WHEEL_ODOMETRY)    {wheelOdometryMsg     = *(WheelOdometryMsg*)msg    ;}    
}

void copyMsg(msg_id msg_id, void *msg){
    if     (msg_id == msg_id::ACTUATOR_OUTPUT)   {*(ActuatorOutputMsg*   ) msg = actuatorOutputMsg   ;}
    else if(msg_id == msg_id::BATTERY_INFO)      {*(BatteryInfoMsg*      ) msg = batteryInfoMsg      ;}
    else if(msg_id == msg_id::CTRL_SETPOINT)     {*(CtrlSetpointMsg*     ) msg = ctrlSetpointMsg     ;}
    else if(msg_id == msg_id::DIAL_POSITION)     {*(DialPositionMsg*     ) msg = dialPositionMsg     ;}
    else if(msg_id == msg_id::GAMEPAD)           {*(GamepadMsg*          ) msg = gamepadMsg          ;}
    else if(msg_id == msg_id::GROUND_TRUTH)      {*(GroundTruthMsg*      ) msg = groundTruthMsg      ;}
    else if(msg_id == msg_id::IMU)               {*(ImuMsg*              ) msg = imuMsg              ;}
    else if(msg_id == msg_id::NAV_STATE)         {*(NavStateMsg*         ) msg = navStateMsg         ;}    
    else if(msg_id == msg_id::VEHICLE_ATTITUDE)  {*(VehicleAttitudeMsg*  ) msg = vehicleAttitudeMsg  ;}
    else if(msg_id == msg_id::VEHICLE_POSITION)  {*(VehiclePositionMsg*  ) msg = vehiclePositionMsg  ;}
    else if(msg_id == msg_id::WALL_SENSOR)       {*(WallSensorMsg*       ) msg = wallSensorMsg       ;}
    else if(msg_id == msg_id::WHEEL_ODOMETRY)    {*(WheelOdometryMsg*    ) msg = wheelOdometryMsg    ;}
}
