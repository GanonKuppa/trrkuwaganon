#include <stdint.h>
#include <string>
#include <map>
#include <typeinfo>

#include "parameterManager.h"
#include "hal_flashRom.h"
#include "hal_timer.h"

#include "ntlibc.h"

namespace module {

    ParameterManager::ParameterManager() {
        registration<float>(0, mass, "mass");
        registration<float>(1, dia_tire, "dia_tire");
        registration<float>(2, tread, "tread");
        registration<uint8_t>(3, dummy0, "dummy0");
        registration<uint16_t>(4, dummy1, "dummy1");
        registration<uint32_t>(5, dummy2, "dummy2");
        registration<int8_t>(6, dummy3, "dummy3");
        registration<int16_t>(7, dummy4, "dummy4");
        registration<int32_t>(8, dummy5, "dummy5");
        
        registration<float>(150, gyro0_x_offset, "gyro0_x_offset"); //150
        registration<float>(151, gyro0_y_offset, "gyro0_y_offset"); //151
        registration<float>(152, gyro0_z_offset, "gyro0_z_offset"); //152
        registration<float>(153, acc0_x_offset, "acc0_x_offset"); //153
        registration<float>(154, acc0_y_offset, "acc0_y_offset"); //154
        registration<float>(155, acc0_z_offset, "acc0_z_offset"); //155
        registration<float>(156, gyro0_x_scaler_cw, "gyro0_x_scaler_cw"); //156
        registration<float>(157, gyro0_x_scaler_ccw, "gyro0_x_scaler_ccw"); //157
        registration<float>(158, gyro0_y_scaler_cw, "gyro0_y_scaler_cw"); //158
        registration<float>(159, gyro0_y_scaler_ccw, "gyro0_y_scaler_ccw"); //159
        registration<float>(160, gyro0_z_scaler_cw, "gyro0_z_scaler_cw"); //160
        registration<float>(161, gyro0_z_scaler_ccw, "gyro0_z_scaler_ccw"); //161        
        registration<float>(162, acc0_x_scaler, "acc0_x_scaler"); //162
        registration<float>(163, acc0_y_scaler, "acc0_y_scaler"); //163
        registration<float>(164, acc0_z_scaler, "acc0_z_scaler"); //164

        registration<float>(165, gyro1_x_offset, "gyro1_x_offset"); //165
        registration<float>(166, gyro1_y_offset, "gyro1_y_offset"); //166
        registration<float>(167, gyro1_z_offset, "gyro1_z_offset"); //167
        registration<float>(168, acc1_x_offset, "acc1_x_offset"); //168
        registration<float>(169, acc1_y_offset, "acc1_y_offset"); //169
        registration<float>(170, acc1_z_offset, "acc1_z_offset"); //170
        registration<float>(171, gyro1_x_scaler_cw, "gyro1_x_scaler_cw"); //171
        registration<float>(172, gyro1_x_scaler_ccw, "gyro1_x_scaler_ccw"); //172
        registration<float>(173, gyro1_y_scaler_cw, "gyro1_y_scaler_cw"); //173
        registration<float>(174, gyro1_y_scaler_ccw, "gyro1_y_scaler_ccw"); //174
        registration<float>(175, gyro1_z_scaler_cw, "gyro1_z_scaler_cw"); //175
        registration<float>(176, gyro1_z_scaler_ccw, "gyro1_z_scaler_ccw"); //176        
        registration<float>(177, acc1_x_scaler, "acc1_x_scaler"); //177
        registration<float>(178, acc1_y_scaler, "acc1_y_scaler"); //178
        registration<float>(179, acc1_z_scaler, "acc1_z_scaler"); //179

        registration<float>(180, heater_p, "heater_p"); //180
        registration<float>(181, heater_p, "heater_i"); //181
        registration<float>(182, heater_i, "heager_i_limit"); //182
        registration<float>(183, heater_i, "heager_limit"); //183
        registration<float>(184, heater_target_temp, "heater_target_temp"); //184

    }

    
//プログラム中の変数にデータフラッシュの保存域を割り当て
//登録時に変数にデータフラッシュに保存されている値を代入
//登録を行った変数はデータフラッシュの保存域を更新する(write関数)際に値を共に変更
    template<typename T>
    void ParameterManager::registration(uint16_t val_num, T& r_val, std::string key) {
        uint16_t index = val_num * 64;
        //uint8_t len = sizeof(T);
        strkeyMap[key] = val_num;
        valStrkeyMap[val_num] = key;
        T* adr = &r_val;
        adrMap[val_num] = reinterpret_cast<uint32_t>(adr);        
        r_val = read<T>(val_num);

        if (typeid(float) == typeid(r_val)) typeMap[val_num] = Type_e::FLOAT;
        if (typeid(uint8_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT8;
        if (typeid(uint16_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT16;
        if (typeid(uint32_t) == typeid(r_val)) typeMap[val_num] = Type_e::UINT32;
        if (typeid(int8_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT8;
        if (typeid(int16_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT16;
        if (typeid(int32_t) == typeid(r_val)) typeMap[val_num] = Type_e::INT32;
    }

    template<typename T>
    bool ParameterManager::write(uint16_t val_num, T val) {
        uint16_t index = val_num * 64;
        bool rtn = false;
        int try_count = 0;
        while (1) {
            if (hal::eraseCheckFlashRom(index, 64) == false) {
                hal::eraseFlashRom(index);
            };
            rtn = hal::writeFlashRom(index, &val, sizeof(T));
            
            if (read<T>(val_num) == val) break;
            PRINTF_ASYNC("  %d |write error!\n", try_count);
            try_count ++;
            if(try_count > 10){
                PRINTF_ASYNC("write error timeout!\n");            
                break;
            }
        }

        //val_numに変数が登録されている場合はその変数を書き換え
        if (adrMap.find(val_num) != adrMap.end()) {
            *reinterpret_cast<T*>(adrMap[val_num]) = val;
            //PRINTF_ASYNC("%d | write f: %f %f", val_num, val, *reinterpret_cast<T*>(adrMap[val_num]));
            //PRINTF_ASYNC("|| write d: %d %d \n", val, *reinterpret_cast<T*>(adrMap[val_num]));
        }
        return rtn;
    }

    template<typename T>
    T ParameterManager::read(uint16_t val_num) {
        T val;
        uint16_t index = val_num * 64;
        hal::readFlashRom(index, &val, sizeof(T));

        return val;
    }


    template<typename T>
    bool ParameterManager::write(std::string key, T val) {
        return write(strkeyMap[key], val);
    }

    template<typename T>
    T ParameterManager::read(std::string key) {
        return read<T>(strkeyMap[key]);
    }



//テンプレートクラスの実体化
    template void ParameterManager::registration(uint16_t val_num, float& r_val, std::string key);
    template void ParameterManager::registration(uint16_t val_num, uint8_t& r_val, std::string key);
    template void ParameterManager::registration(uint16_t val_num, uint16_t& r_val, std::string key);
    template void ParameterManager::registration(uint16_t val_num, uint32_t& r_val, std::string key);
    template void ParameterManager::registration(uint16_t val_num, int8_t& r_val, std::string key);
    template void ParameterManager::registration(uint16_t val_num, int16_t& r_val, std::string key);
    template void ParameterManager::registration(uint16_t val_num, int32_t& r_val, std::string key);

    template bool ParameterManager::write(uint16_t val_num, float val);
    template bool ParameterManager::write(uint16_t val_num, uint8_t val);
    template bool ParameterManager::write(uint16_t val_num, uint16_t val);
    template bool ParameterManager::write(uint16_t val_num, uint32_t val);
    template bool ParameterManager::write(uint16_t val_num, int8_t val);
    template bool ParameterManager::write(uint16_t val_num, int16_t val);
    template bool ParameterManager::write(uint16_t val_num, int32_t val);

    template float ParameterManager::read<float>(uint16_t val_num);
    template uint8_t ParameterManager::read<uint8_t>(uint16_t val_num);
    template uint16_t ParameterManager::read<uint16_t>(uint16_t val_num);
    template uint32_t ParameterManager::read<uint32_t>(uint16_t val_num);
    template int8_t ParameterManager::read<int8_t>(uint16_t val_num);
    template int16_t ParameterManager::read<int16_t>(uint16_t val_num);
    template int32_t ParameterManager::read<int32_t>(uint16_t val_num);

    template bool ParameterManager::write(std::string key, float val);
    template bool ParameterManager::write(std::string key, uint8_t val);
    template bool ParameterManager::write(std::string key, uint16_t val);
    template bool ParameterManager::write(std::string key, uint32_t val);
    template bool ParameterManager::write(std::string key, int8_t val);
    template bool ParameterManager::write(std::string key, int16_t val);
    template bool ParameterManager::write(std::string key, int32_t val);

    template float ParameterManager::read<float>(std::string key);
    template uint8_t ParameterManager::read<uint8_t>(std::string key);
    template uint16_t ParameterManager::read<uint16_t>(std::string key);
    template uint32_t ParameterManager::read<uint32_t>(std::string key);
    template int8_t ParameterManager::read<int8_t>(std::string key);
    template int16_t ParameterManager::read<int16_t>(std::string key);
    template int32_t ParameterManager::read<int32_t>(std::string key);

    int usrcmd_parameterManager(int argc, char **argv){

        if (ntlibc_strcmp(argv[1], "list") == 0) {
            PRINTF_ASYNC("  # no  , type   , name_str        , val\n");                           
            auto &pm = ParameterManager::getInstance();
            for(int i=0;i<pm.valStrkeyMap.size();i++){

                uint16_t val_num = i;
                std::string key_str = pm.valStrkeyMap[val_num];
                Type_e val_type = pm.typeMap[val_num];
                
                if(val_type == Type_e::FLOAT){
                    std::string type_str = "float";
                    float val = pm.read<float>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %f\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else if(val_type == Type_e::UINT8){
                    std::string type_str = "uint8";
                    uint8_t val = pm.read<uint8_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else if(val_type == Type_e::UINT16){
                    std::string type_str = "uint16";
                    uint16_t val = pm.read<uint16_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else if(val_type == Type_e::UINT32){
                    std::string type_str = "uint32";
                    uint32_t val = pm.read<uint32_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else if(val_type == Type_e::INT8){
                    std::string type_str = "int8";
                    int8_t val = pm.read<int8_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else if(val_type == Type_e::INT16){
                    std::string type_str = "int16";
                    int16_t val = pm.read<int16_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else if(val_type == Type_e::INT32){
                    std::string type_str = "int32";
                    int32_t val = pm.read<int32_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), key_str. c_str(), val);
                }
                else{
                    PRINTF_ASYNC("  invalid type!\n");                
                }            

                hal::waitmsec(1);
            }
            return 0;
        }

        if(ntlibc_strcmp(argv[1], "ttl") == 0){
            PRINTF_ASYNC("\n");                           
            PRINTF_ASYNC(";-------- teraterm macro file --------\n");                           
            auto &pm = ParameterManager::getInstance();
            for(int i=0;i<pm.valStrkeyMap.size();i++){

                uint16_t val_num = i;
                std::string key_str = pm.valStrkeyMap[val_num];
                Type_e val_type = pm.typeMap[val_num];
                
                if(val_type == Type_e::FLOAT){
                    std::string type_str = "float";
                    float val = pm.read<float>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %f\' \n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else if(val_type == Type_e::UINT8){
                    std::string type_str = "uint8";
                    uint8_t val = pm.read<uint8_t>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %d\'\n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else if(val_type == Type_e::UINT16){
                    std::string type_str = "uint16";
                    uint16_t val = pm.read<uint16_t>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %d\'\n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else if(val_type == Type_e::UINT32){
                    std::string type_str = "uint32";
                    uint32_t val = pm.read<uint32_t>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %d\'\n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else if(val_type == Type_e::INT8){
                    std::string type_str = "int8";
                    int8_t val = pm.read<int8_t>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %d\'\n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else if(val_type == Type_e::INT16){
                    std::string type_str = "int16";
                    int16_t val = pm.read<int16_t>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %d\'\n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else if(val_type == Type_e::INT32){
                    std::string type_str = "int32";
                    int32_t val = pm.read<int32_t>(val_num);                
                    PRINTF_ASYNC("; %3d | %-7s\n", val_num, type_str.c_str());
                    PRINTF_ASYNC("sendln \'param write %-20s %d'\n", key_str.c_str(), val);
                    PRINTF_ASYNC("wait \'>\'\n");
                }
                else{
                    PRINTF_ASYNC(";invalid type!\n");                
                }
                hal::waitmsec(1);
            }
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "write") == 0 ) {
            if(argc != 4){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            std::string param_name_str(argv[2]);
            std::string param_val_str(argv[3]);
            auto &pm = ParameterManager::getInstance();
            if (pm.strkeyMap.find(param_name_str) == pm.strkeyMap.end()) {
                PRINTF_ASYNC("  %s | parameter not found!\n", param_name_str.c_str());
            }
            else{
                uint16_t val_num = pm.strkeyMap[param_name_str];
                Type_e val_type = pm.typeMap[val_num];

                if(val_type == Type_e::FLOAT){
                    float param_val = std::stof(param_val_str);
                    std::string type_str = "float";
                    pm.write<float>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %f\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::UINT8){
                    uint8_t param_val = std::stoi(param_val_str);
                    std::string type_str = "uint8_t";
                    pm.write<uint8_t>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::UINT16){
                    uint16_t param_val = std::stoi(param_val_str);
                    std::string type_str = "uint16_t";
                    pm.write<uint16_t>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::UINT32){
                    uint32_t param_val = std::stoi(param_val_str);
                    std::string type_str = "uint32_t";
                    pm.write<uint32_t>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::INT8){
                    int8_t param_val = std::stoi(param_val_str);
                    std::string type_str = "int8_t";
                    pm.write<int8_t>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::INT16){
                    int16_t param_val = std::stoi(param_val_str);
                    std::string type_str = "int16_t";
                    pm.write<int16_t>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::INT32){
                    int32_t param_val = std::stoi(param_val_str);
                    std::string type_str = "int32_t";
                    pm.write<int32_t>(val_num, param_val);
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else{
                    PRINTF_ASYNC("  invalid type!\n");
                    return -1;
                }                        
            }
            return 0;
        }
        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;

    };


}

