#pragma once

enum class msg_id{
    BATTERY_INFO = 0,
    CTRL_SETPOINT,
    GAMEPAD,    
    IMU,
    WALL_SENSOR,
    WHEEL_ODOMETRY,
    POSITION_ESTIMATOR,
    GROUND_TRUTH
};

void publishMsg(msg_id msg_id, void* msg);
void copyMsg(msg_id msg_id, void* msg);
