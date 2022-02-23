#pragma once

#include <stdint.h>

#include "baseModule.h"
#include "turnEnum.h"


namespace module{

    class TrajInfo{
      public:
        ETurnType turn_type_next = ETurnType::NONE;
        ETurnDir turn_dir_next = ETurnDir::NO_TURN;
        int16_t wall_sensor_max = 0;
        int16_t wall_sensor_min = 32767;
        float end_yaw;               
    };

    class RunAnalizer : public BaseModule<RunAnalizer>{
      public:    
        void update0();
        void debug();
        void print();
      private:        
        static constexpr uint16_t TRAJ_INFO_MAX = 100;
        TrajInfo _traj_info_list[TRAJ_INFO_MAX];        
        uint16_t _traj_info_num;
        bool _in_detect_edge_area_pre;
        friend class BaseModule<RunAnalizer>;
        RunAnalizer();
    };

    int usrcmd_runAnalizer(int argc, char **argv);

}
