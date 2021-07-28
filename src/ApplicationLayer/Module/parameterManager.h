#pragma once

#include <stdint.h>
#include <string>
#include <map>

#include "baseModule.h"

namespace module {

    enum struct Type_e {
        FLOAT =0,
        UINT8,
        UINT16,
        UINT32,
        INT8,
        INT16,
        INT32
    };

    class ParameterManager : public BaseModule<ParameterManager> {    
      public:
        std::map<uint16_t, uint32_t> adrMap;
        std::map<uint16_t, Type_e> typeMap;
        std::map<std::string, uint16_t> strkeyMap;
        std::map<uint16_t, std::string> valStrkeyMap;

        template<typename T>
        bool write(uint16_t val_num, T val);

        template<typename T>
        T read(uint16_t val_num);

        template<typename T>
        bool write(std::string key, T val);

        template<typename T>
        T read(std::string key);

        template<typename T>
        void registration(uint16_t val_num, T& r_val, std::string key);

        //----管理下においた変数たち-----
        float mass; //0
        float dia_tire; //1
        float tread; //2
        float duty_limit;
        uint16_t dummy1;
        uint32_t dummy2;
        int8_t dummy3;
        int16_t dummy4;
        int32_t dummy5;


        float gyro0_x_offset; //150
        float gyro0_y_offset; //151
        float gyro0_z_offset; //152
        float acc0_x_offset; //153
        float acc0_y_offset; //154
        float acc0_z_offset; //155
        float gyro0_x_scaler_cw; //156
        float gyro0_x_scaler_ccw; //157
        float gyro0_y_scaler_cw; //158
        float gyro0_y_scaler_ccw; //159
        float gyro0_z_scaler_cw; //160
        float gyro0_z_scaler_ccw; //161        
        float acc0_x_scaler; //162
        float acc0_y_scaler; //163
        float acc0_z_scaler; //164        

        float gyro1_x_offset; //165
        float gyro1_y_offset; //166
        float gyro1_z_offset; //167
        float acc1_x_offset; //168
        float acc1_y_offset; //169
        float acc1_z_offset; //170
        float gyro1_x_scaler_cw; //171
        float gyro1_x_scaler_ccw; //172
        float gyro1_y_scaler_cw; //173
        float gyro1_y_scaler_ccw; //174
        float gyro1_z_scaler_cw; //175
        float gyro1_z_scaler_ccw; //176        
        float acc1_x_scaler; //177
        float acc1_y_scaler; //178
        float acc1_z_scaler; //179        
        
        float heater_p; //180
        float heater_i; //181
        float heater_i_limit; //182
        float heater_limit; //183
        float heater_target_temp; //184


        //----管理下においた変数たち-----

      private:
        friend class BaseModule<ParameterManager>;
        ParameterManager();
    };

    int usrcmd_parameterManager(int argc, char **argv);

}
