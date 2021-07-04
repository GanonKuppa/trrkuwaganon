#include <stdint.h>
#include <string>
#include <map>
#include <typeinfo>

#include "parameterManager.h"
#include "hal_flashRom.h"

namespace module {

    ParameterManager::ParameterManager() {
        registration<float>(0, mass, "mass");
        registration<float>(1, dia_tire, "dia_tire");
        registration<float>(2, tread, "tread");
    }

//プログラム中の変数にデータフラッシュの保存域を割り当て
//登録時に変数にデータフラッシュに保存されている値を代入
//登録を行った変数はデータフラッシュの保存域を更新する(write関数)際に値を共に変更
    template<typename T>
    void ParameterManager::registration(uint16_t val_num, T& r_val, std::string key) {
        uint16_t index = val_num * 64;
        //uint8_t len = sizeof(T);
        strkeyMap[key] = val_num;
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
        bool rtn;
        while (1) {
            if (hal::eraseCheckFlashRom(index, 64) == false) {
                hal::eraseFlashRom(index);
            };
            rtn = hal::writeFlashRom(index, &val, sizeof(T));
            //printfAsync("write error!\n");
            if (read<T>(val_num) == val) break;
        }

        //val_numに変数が登録されている場合はその変数を書き換え
        if (adrMap.find(val_num) != adrMap.end()) {
            *reinterpret_cast<T*>(adrMap[val_num]) = val;
            //printfAsync("%d | write f: %f %f", val_num, val, *reinterpret_cast<T*>(adrMap[val_num]));
            //printfAsync("|| write d: %d %d \n", val, *reinterpret_cast<T*>(adrMap[val_num]));
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

}

