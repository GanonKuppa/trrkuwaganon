#pragma once

enum class msg_id{
    ACTUATOR_OUTPUT =0,
    BATTERY_INFO,
    CTRL_SETPOINT,
    DIAL_POSITION,
    GAMEPAD,
    GROUND_TRUTH,
    IMU,
    NAV_STATE,    
    VEHICLE_ATTITUDE,
    VEHICLE_POSITION,
    WALL_SENSOR,
    WHEEL_ODOMETRY            
};

void publishMsg(msg_id msg_id, void* msg);
void copyMsg(msg_id msg_id, void* msg);
