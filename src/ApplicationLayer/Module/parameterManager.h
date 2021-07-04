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
        //----管理下においた変数たち-----

      private:
        friend class BaseModule<ParameterManager>;
        ParameterManager();
    };

}
