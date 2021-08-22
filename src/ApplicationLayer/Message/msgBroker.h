#pragma once

enum class msg_id{
    BATTERY_INFO = 0,
    CTRL_SETPOINT,
    ACTUATOR_OUTPUT,
    IMU,
    WHEEL_ODOMETRY,
    NAV_STATE,
    WALL_SENSOR,
    VEHICLE_POSITION,
    VEHICLE_ATTITUDE,
    GROUND_TRUTH,
    GAMEPAD
};

void publishMsg(msg_id msg_id, void* msg);
void copyMsg(msg_id msg_id, void* msg);
