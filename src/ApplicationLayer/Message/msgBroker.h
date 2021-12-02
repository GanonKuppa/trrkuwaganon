#pragma once

#include <stdint.h>

enum class msg_id : uint8_t{
    ACTUATOR_OUTPUT =0,
    BATTERY_INFO,
    CTRL_SETPOINT,
    DIAL_POSITION,
    GAMEPAD,
    GROUND_TRUTH,
    IMU,
    NAV_STATE,
    PID_CONTROL_VAL, 
    VEHICLE_ATTITUDE,
    VEHICLE_POSITION,
    WALL_SENSOR,
    WHEEL_ODOMETRY            
};

void publishMsg(msg_id msg_id, void* msg);
void copyMsg(msg_id msg_id, void* msg);
