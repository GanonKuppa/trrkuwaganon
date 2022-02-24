#include "runAnalizer.h"

#include <string>
#include <algorithm>

// Hal
#include "hal_timer.h"
#include "hal_uart.h"

// Lib
#include "debugLog.h"
#include "ntlibc.h"

// Msg
#include "msgBroker.h"

#include "ctrlSetpointMsg.h"
#include "wallSensorMsg.h"
#include "vehicleAttitudeMsg.h"
#include "vehiclePositionMsg.h"
#include "navStateMsg.h"
#include "trajTripletMsg.h"
#include "parameterManager.h"


namespace module {

RunAnalizer::RunAnalizer() :
	_traj_info_num(0),
    _in_detect_edge_area_pre(false),
    _detected_edge_pre(false)
	{
		setModuleName("RunAnalizer");
        for(uint8_t i=0;i<TRAJ_INFO_MAX;i++){
            _traj_info_list[i].turn_type_next = ETurnType::NONE;
            _traj_info_list[i].turn_dir_next = ETurnDir::NO_TURN;
            _traj_info_list[i].wall_sensor_max = 0;
            _traj_info_list[i].wall_sensor_min = 32767;
            _traj_info_list[i].end_x = 0.0f;
            _traj_info_list[i].detected_x = 0.0f;
            _traj_info_list[i].end_y = 0.0f;
            _traj_info_list[i].detected_y = 0.0f;
            _traj_info_list[i].end_yaw = 0.0f;
        }
	}


    void RunAnalizer::print(){
        ParameterManager& pm = ParameterManager::getInstance();
        PRINTF_ASYNC("  ---- wall_corner_threshold ---- \n")
        PRINTF_ASYNC("  wall_corner_threshold_on_r  : %8d\n", pm.wall_corner_threshold_on_r);
        PRINTF_ASYNC("  wall_corner_threshold_off_r : %8d\n", pm.wall_corner_threshold_off_r);
        PRINTF_ASYNC("  wall_corner_threshold_on_l  : %8d\n", pm.wall_corner_threshold_on_l);
        PRINTF_ASYNC("  wall_corner_threshold_off_l : %8d\n", pm.wall_corner_threshold_off_l);
        PRINTF_ASYNC("  ---- diag_corner_threshold ---- \n");
        PRINTF_ASYNC("  diag_corner_threshold_on_r  : %8d\n", pm.diag_corner_threshold_on_r);
        PRINTF_ASYNC("  diag_corner_threshold_off_r : %8d\n", pm.diag_corner_threshold_off_r);
        PRINTF_ASYNC("  diag_corner_threshold_on_l  : %8d\n", pm.wall_corner_threshold_on_l);
        PRINTF_ASYNC("  diag_corner_threshold_off_l : %8d\n", pm.wall_corner_threshold_off_l);

        PRINTF_ASYNC("  ------------------------------ \n");
        PRINTF_ASYNC("  num, turn_type_next, dir, wall_max, wall_min,  end_yaw\n");
        for(uint32_t i=0; i<_traj_info_num; i++){    
            PRINTF_ASYNC("  %03d, %-14s, %3s, %8d, %8d, %8d\n",
              i,
              turnType2Str(_traj_info_list[i].turn_type_next).c_str(),
              turnDir2Str(_traj_info_list[i].turn_dir_next).c_str(),
              _traj_info_list[i].wall_sensor_max,
              _traj_info_list[i].wall_sensor_min,
              (int16_t)(_traj_info_list[i].end_yaw * 180.0f /3.14159265)
            );
            PRINTF_ASYNC("\n");
            PRINTF_ASYNC("    x | %7.3f, %7.3f | %7.3f\n", _traj_info_list[i].end_x, _traj_info_list[i].detected_x, _traj_info_list[i].end_x - _traj_info_list[i].detected_x);
            PRINTF_ASYNC("    y | %7.3f, %7.3f | %7.3f\n", _traj_info_list[i].end_y, _traj_info_list[i].detected_y, _traj_info_list[i].end_y - _traj_info_list[i].detected_y);

            hal::waitmsec(10);
        }
    }

    void RunAnalizer::debug(){
        PRINTF_ASYNC("  ----- runAnalizer status -----\n");
        PRINTF_ASYNC("  traj_info_num : %d\n", _traj_info_num);
    }

    void RunAnalizer::update0(){
        CtrlSetpointMsg ctrl_msg;
        WallSensorMsg ws_msg;
        VehicleAttitudeMsg att_msg;
        VehiclePositionMsg pos_msg;
        NavStateMsg nav_msg;
        TrajTripletMsg traj_msg;

        copyMsg(msg_id::CTRL_SETPOINT, &ctrl_msg);
        copyMsg(msg_id::WALL_SENSOR, &ws_msg);
        copyMsg(msg_id::VEHICLE_ATTITUDE, &att_msg);
        copyMsg(msg_id::VEHICLE_POSITION, &pos_msg);
        copyMsg(msg_id::NAV_STATE, &nav_msg);
        copyMsg(msg_id::TRAJ_TRIPLET, &traj_msg);                                 

        if(ctrl_msg.in_detect_edge_area){
            _traj_info_list[_traj_info_num].turn_type_next = traj_msg.turn_type_next;
            _traj_info_list[_traj_info_num].turn_dir_next = traj_msg.turn_dir_next;

            if(traj_msg.turn_dir_next == ETurnDir::CCW && ws_msg.dist_a > 0.09f){
            	_traj_info_list[_traj_info_num].wall_sensor_max = std::max(ws_msg.left, _traj_info_list[_traj_info_num].wall_sensor_max);
                _traj_info_list[_traj_info_num].wall_sensor_min = std::min(ws_msg.left, _traj_info_list[_traj_info_num].wall_sensor_min);
            }

            else if(traj_msg.turn_dir_next == ETurnDir::CW && ws_msg.dist_a > 0.09f){
                _traj_info_list[_traj_info_num].wall_sensor_max = std::max(ws_msg.right, _traj_info_list[_traj_info_num].wall_sensor_max);
                _traj_info_list[_traj_info_num].wall_sensor_min = std::min(ws_msg.right, _traj_info_list[_traj_info_num].wall_sensor_min);
            }
            _traj_info_list[_traj_info_num].end_yaw = traj_msg.end_yaw_now;
            _traj_info_list[_traj_info_num].end_x = traj_msg.end_x_now;
            _traj_info_list[_traj_info_num].end_y = traj_msg.end_y_now;

            if(!_detected_edge_pre && ctrl_msg.detected_edge){
                _traj_info_list[_traj_info_num].detected_x = pos_msg.x;
                _traj_info_list[_traj_info_num].detected_y = pos_msg.y;
            }
        }

        if(_in_detect_edge_area_pre && !ctrl_msg.in_detect_edge_area && _traj_info_num < TRAJ_INFO_MAX){
            _traj_info_num ++;
        }
        
        _in_detect_edge_area_pre = ctrl_msg.in_detect_edge_area;
        _detected_edge_pre = ctrl_msg.detected_edge;
    }

    int usrcmd_runAnalizer(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "help") == 0) {
            PRINTF_ASYNC("  status :\r\n");
            PRINTF_ASYNC("  print  :\r\n");
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            RunAnalizer::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "print") == 0){
            PRINTF_ASYNC("------------------------------------------\n");
            RunAnalizer::getInstance().print();
            return 0;
        }

        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;

    }

}

