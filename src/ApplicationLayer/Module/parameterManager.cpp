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
        setModuleName("ParameterManager");
#ifdef SILS
        return;
#endif

        registration<float>(0, mass, "mass"); //0
        registration<float>(1, dia_tire, "dia_tire"); //1
        registration<float>(2, tread, "tread"); //2
        registration<float>(3, duty_limit, "duty_limit"); //3
        registration<uint8_t>(4, silent_flag, "silent_flag"); //4
        registration<float>(5, test_run_v, "test_run_v"); //5
        registration<float>(6, test_run_a, "test_run_a"); //6
        registration<float>(7, test_run_x, "test_run_x"); //7
        registration<uint8_t>(8, test_run_wall_flag, "test_run_wall_flag"); //8
        registration<float>(9, v_search_run, "v_search_run"); //9
        registration<float>(10, a_search_run, "a_search_run"); //10
        registration<float>(11, spin_yawrate_max, "spin_yawrate_max"); //11
        registration<float>(12, spin_yawacc, "spin_yawacc"); //12
        registration<uint8_t>(13, goal_x, "goal_x"); //13
        registration<uint8_t>(14, goal_y, "goal_y"); //14
        registration<float>(15, search_limit_time_sec, "search_limit_time_sec"); //15
        registration<uint8_t>(16, logger_skip_mod, "logger_skip_mod");  //16
        ;//17
        ;//18
        ;//19
        ;//20
        ;//21
        ;//22
        ;//23
        ;//24
        ;//25
        ;//26
        ;//27
        ;//28
        ;//29
        registration<float>(30, duty_coef_left_p, "duty_coef_left_p"); //30
        registration<float>(31, duty_offset_left_p, "duty_offset_left_p"); //31
        registration<float>(32, duty_coef_right_p, "duty_coef_right_p"); //32
        registration<float>(33, duty_offset_right_p, "duty_offset_right_p"); //33
        registration<float>(34, duty_coef_left_m, "duty_coef_left_m"); //34
        registration<float>(35, duty_offset_left_m, "duty_offset_left_m"); //35
        registration<float>(36, duty_coef_right_m, "duty_coef_right_m"); //36
        registration<float>(37, duty_offset_right_m, "duty_offset_right_m"); //37
        ;//38
        ;//39
        ;//40
        ;//41
        ;//42
        ;//43
        ;//44
        ;//45
        ;//46
        ;//47
        ;//48
        ;//49
        registration<uint8_t>(50, v_fb_enable, "v_fb_enable"); //50
        registration<uint8_t>(51, v_ff_enable, "v_ff_enable"); //51
        registration<uint8_t>(52, v_saturation_enable, "v_saturation_enable"); //52
        registration<uint8_t>(53, v_i_saturation_enable, "v_i_saturation_enable"); //53
        registration<uint8_t>(54, yawrate_fb_enable, "yawrate_fb_enable"); //54
        registration<uint8_t>(55, yawrate_ff_enable, "yawrate_ff_enable"); //55
        registration<uint8_t>(56, yawrate_saturation_enable, "yawrate_saturation_enable"); //56
        registration<uint8_t>(57, yawrate_i_saturation_enable, "yawrate_i_saturation_enable"); //57
        registration<uint8_t>(58, yaw_fb_enable, "yaw_fb_enable"); //58
        registration<uint8_t>(59, yaw_saturation_enable, "yaw_saturation_enable"); //59
        registration<uint8_t>(60, yaw_i_saturation_enable, "yaw_i_saturation_enable"); //60
        registration<uint8_t>(61, wall_fb_enable, "wall_fb_enable"); //61
        registration<uint8_t>(62, wall_saturation_enable, "wall_saturation_enable"); //62
        registration<uint8_t>(63, wall_i_saturation_enable, "wall_i_saturation_enable"); //63
        registration<uint8_t>(64, wall_diag_fb_enable, "wall_diag_fb_enable"); //64
        registration<uint8_t>(65, wall_diag_saturation_enable, "wall_diag_saturation_enable"); //65
        registration<uint8_t>(66, wall_diag_i_saturation_enable, "wall_diag_i_saturation_enable"); //66
        ;//67
        ;//68
        ;//69
        ;//70
        ;//71
        ;//72
        ;//73
        ;//74
        ;//75
        ;//76
        ;//77
        ;//78
        ;//79
        registration<float>(80, v_ff_coef, "v_ff_coef"); //80
        registration<float>(81, v_ff_offset, "v_ff_offset"); //81
        registration<float>(82, a_ff_coef, "a_ff_coef"); //82
        registration<float>(83, a_ff_offset, "a_ff_offset"); //83
        registration<float>(84, yawrate_ff_coef, "yawrate_ff_coef"); //84
        registration<float>(85, yawrate_ff_offset, "yawrate_ff_offset"); //85
        registration<float>(86, yawacc_ff_coef, "yawacc_ff_coef"); //86
        registration<float>(87, yawacc_ff_offset, "yawacc_ff_offset"); //87
        registration<float>(88, search_v_p, "search_v_p"); //88
        registration<float>(89, search_v_i, "search_v_i"); //89
        registration<float>(90, search_v_d, "search_v_d"); //90
        registration<float>(91, search_v_f, "search_v_f"); //91
        registration<float>(92, search_yawrate_p, "search_yawrate_p"); //92
        registration<float>(93, search_yawrate_i, "search_yawrate_i"); //93
        registration<float>(94, search_yawrate_d, "search_yawrate_d"); //94
        registration<float>(95, search_yawrate_f, "search_yawrate_f"); //95
        registration<float>(96, search_yaw_p, "search_yaw_p"); //96
        registration<float>(97, search_yaw_i, "search_yaw_i"); //97
        registration<float>(98, fast_v_p, "fast_v_p"); //98
        registration<float>(99, fast_v_i, "fast_v_i"); //99
        registration<float>(100, fast_v_d, "fast_v_d"); //100
        registration<float>(101, fast_v_f, "fast_v_f"); //101
        registration<float>(102, fast_yawrate_p, "fast_yawrate_p"); //102
        registration<float>(103, fast_yawrate_i, "fast_yawrate_i"); //103
        registration<float>(104, fast_yawrate_d, "fast_yawrate_d"); //104
        registration<float>(105, fast_yawrate_f, "fast_yawrate_f"); //105
        registration<float>(106, fast_yaw_p, "fast_yaw_p"); //106
        registration<float>(107, fast_yaw_i, "fast_yaw_i"); //107
        registration<float>(108, spin_v_p, "spin_v_p"); //108
        registration<float>(109, spin_v_i, "spin_v_i"); //109
        registration<float>(110, spin_v_d, "spin_v_d"); //110
        registration<float>(111, spin_v_f, "spin_v_f"); //111
        registration<float>(112, spin_yawrate_p, "spin_yawrate_p"); //112
        registration<float>(113, spin_yawrate_i, "spin_yawrate_i"); //113
        registration<float>(114, spin_yawrate_d, "spin_yawrate_d"); //114
        registration<float>(115, spin_yawrate_f, "spin_yawrate_f"); //115
        registration<float>(116, spin_yaw_p, "spin_yaw_p"); //116
        registration<float>(117, spin_yaw_i, "spin_yaw_i"); //117
        registration<float>(118, wall_p, "wall_p"); //118
        registration<float>(119, wall_i, "wall_i"); //119
        registration<float>(120, wall_d, "wall_d"); //120
        registration<float>(121, wall_f, "wall_f"); //121
        registration<float>(122, wall_diag_p, "wall_diag_p"); //122
        registration<float>(123, wall_diag_i, "wall_diag_i"); //123
        registration<float>(124, wall_diag_d, "wall_diag_d"); //124
        registration<float>(125, wall_diag_f, "wall_diag_f"); //125
        registration<float>(126, v_saturation_offset_duty, "v_saturation_offset_duty"); //126
        registration<float>(127, v_saturation_ff_multiplier, "v_saturation_ff_multiplier"); //127
        registration<float>(128, yawrate_saturation_offset_duty, "yawrate_saturation_offset_duty"); //128
        registration<float>(129, yawrate_saturation_ff_multiplier, "yawrate_saturation_ff_multiplier"); //129
        registration<float>(130, yaw_saturation, "yaw_saturation"); //130
        registration<float>(131, wall_saturation, "wall_saturation"); //131
        registration<float>(132, wall_diag_saturation, "wall_diag_saturation"); //132
        registration<float>(133, v_i_saturation, "v_i_saturation"); //133
        registration<float>(134, yawrate_i_saturation, "yawrate_i_saturation"); //134
        registration<float>(135, yaw_i_saturation, "yaw_i_saturation"); //135
        registration<float>(136, wall_i_saturation, "wall_i_saturation"); //136
        registration<float>(137, wall_diag_i_saturation, "wall_diag_i_saturation"); //137
        registration<float>(138, wall_corner_read_offset_r, "wall_corner_read_offset_r"); //138
        registration<float>(139, wall_corner_read_offset_l, "wall_corner_read_offset_l"); //139
        registration<uint16_t>(140, wall_corner_threshold_on_r, "wall_corner_threshold_on_r"); //140
        registration<uint16_t>(141, wall_corner_threshold_off_r, "wall_corner_threshold_off_r"); //141
        registration<uint16_t>(142, wall_corner_threshold_on_l, "wall_corner_threshold_on_l"); //142
        registration<uint16_t>(143, wall_corner_threshold_off_l, "wall_corner_threshold_off_l"); //143
        registration<float>(144, diag_r_corner_read_offset, "diag_r_corner_read_offset"); //144
        registration<float>(145, diag_l_corner_read_offset, "diag_l_corner_read_offset"); //145
        registration<uint16_t>(146, diag_corner_threshold_on_r, "diag_corner_threshold_on_r"); //146
        registration<uint16_t>(147, diag_corner_threshold_off_r, "diag_corner_threshold_off_r"); //147
        registration<uint16_t>(148, diag_corner_threshold_on_l, "diag_corner_threshold_on_l"); //148
        registration<uint16_t>(149, diag_corner_threshold_off_l, "diag_corner_threshold_off_l"); //149
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
        registration<float>(181, heater_i, "heater_i"); //181
        registration<float>(182, heater_i_limit, "heater_i_limit"); //182
        registration<float>(183, heater_limit, "heater_limit"); //183
        registration<float>(184, heater_target_temp, "heater_target_temp"); //184
        registration<float>(185, dial_p, "dial_p"); //185
        registration<float>(186, dial_i, "dial_i"); //186
        registration<float>(187, dial_i_limit, "dial_i_limit"); //187
        registration<float>(188, dial_limit, "dial_limit"); //188
        registration<float>(189, cp_coef, "cp_coef"); // 189
        ;//190
        ;//191
        ;//192
        ;//193
        ;//194
        ;//195
        ;//196
        ;//197
        ;//198
        ;//199
        registration<float>(200, shortest_0_v, "shortest_0_v"); //200
        registration<float>(201, shortest_0_v_d, "shortest_0_v_d"); //201
        registration<float>(202, shortest_0_v_90, "shortest_0_v_90"); //202
        registration<float>(203, shortest_0_v_l90, "shortest_0_v_l90"); //203
        registration<float>(204, shortest_0_v_180, "shortest_0_v_180"); //204
        registration<float>(205, shortest_0_v_d90, "shortest_0_v_d90"); //205
        registration<float>(206, shortest_0_v_45, "shortest_0_v_45"); //206
        registration<float>(207, shortest_0_v_135, "shortest_0_v_135"); //207
        registration<float>(208, shortest_0_a, "shortest_0_a"); //208
        registration<float>(209, shortest_0_a_diag, "shortest_0_a_diag"); //209
        ;//210
        ;//211
        ;//212
        ;//213
        ;//214
        ;//215
        ;//216
        ;//217
        ;//218
        ;//219
        registration<float>(220, shortest_1_v, "shortest_1_v"); //220
        registration<float>(221, shortest_1_v_d, "shortest_1_v_d"); //221
        registration<float>(222, shortest_1_v_90, "shortest_1_v_90"); //222
        registration<float>(223, shortest_1_v_l90, "shortest_1_v_l90"); //223
        registration<float>(224, shortest_1_v_180, "shortest_1_v_180"); //224
        registration<float>(225, shortest_1_v_d90, "shortest_1_v_d90"); //225
        registration<float>(226, shortest_1_v_45, "shortest_1_v_45"); //226
        registration<float>(227, shortest_1_v_135, "shortest_1_v_135"); //227
        registration<float>(228, shortest_1_a, "shortest_1_a"); //228
        registration<float>(229, shortest_1_a_diag, "shortest_1_a_diag"); //229
        ;//230
        ;//231
        ;//232
        ;//233
        ;//234
        ;//235
        ;//236
        ;//237
        ;//238
        ;//239
        registration<float>(240, shortest_2_v, "shortest_2_v"); //240
        registration<float>(241, shortest_2_v_d, "shortest_2_v_d"); //241
        registration<float>(242, shortest_2_v_90, "shortest_2_v_90"); //242
        registration<float>(243, shortest_2_v_l90, "shortest_2_v_l90"); //243
        registration<float>(244, shortest_2_v_180, "shortest_2_v_180"); //244
        registration<float>(245, shortest_2_v_d90, "shortest_2_v_d90"); //245
        registration<float>(246, shortest_2_v_45, "shortest_2_v_45"); //246
        registration<float>(247, shortest_2_v_135, "shortest_2_v_135"); //247
        registration<float>(248, shortest_2_a, "shortest_2_a"); //248
        registration<float>(249, shortest_2_a_diag, "shortest_2_a_diag"); //249
        ;//250
        ;//251
        ;//252
        ;//253
        ;//254
        ;//255
        ;//256
        ;//257
        ;//258
        ;//259
        registration<float>(260, shortest_3_v, "shortest_3_v"); //260
        registration<float>(261, shortest_3_v_d, "shortest_3_v_d"); //261
        registration<float>(262, shortest_3_v_90, "shortest_3_v_90"); //262
        registration<float>(263, shortest_3_v_l90, "shortest_3_v_l90"); //263
        registration<float>(264, shortest_3_v_180, "shortest_3_v_180"); //264
        registration<float>(265, shortest_3_v_d90, "shortest_3_v_d90"); //265
        registration<float>(266, shortest_3_v_45, "shortest_3_v_45"); //266
        registration<float>(267, shortest_3_v_135, "shortest_3_v_135"); //267
        registration<float>(268, shortest_3_a, "shortest_3_a"); //268
        registration<float>(269, shortest_3_a_diag, "shortest_3_a_diag"); //269
        ;//270
        ;//271
        ;//272
        ;//273
        ;//274
        ;//275
        ;//276
        ;//277
        ;//278
        ;//279
        registration<float>(280, shortest_4_v, "shortest_4_v"); //280
        registration<float>(281, shortest_4_v_d, "shortest_4_v_d"); //281
        registration<float>(282, shortest_4_v_90, "shortest_4_v_90"); //282
        registration<float>(283, shortest_4_v_l90, "shortest_4_v_l90"); //283
        registration<float>(284, shortest_4_v_180, "shortest_4_v_180"); //284
        registration<float>(285, shortest_4_v_d90, "shortest_4_v_d90"); //285
        registration<float>(286, shortest_4_v_45, "shortest_4_v_45"); //286
        registration<float>(287, shortest_4_v_135, "shortest_4_v_135"); //287
        registration<float>(288, shortest_4_a, "shortest_4_a"); //288
        registration<float>(289, shortest_4_a_diag, "shortest_4_a_diag"); //289
        
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

#ifndef SILS
        adrMap[val_num] = reinterpret_cast<uint32_t>(adr);
#else
        adrMap[val_num] = reinterpret_cast<uint64_t>(adr);
#endif
        *reinterpret_cast<T*>(adrMap[val_num]) = read<T>(val_num);

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
                
                if(key_str.empty()){
                    PRINTF_ASYNC("    %3d\n", val_num);
                }
                else if(val_type == Type_e::FLOAT){
                    std::string type_str = "float";
                    float val_from_adr = *reinterpret_cast<float*>(pm.adrMap[val_num]);
                    float val = pm.read<float>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %f, %f\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else if(val_type == Type_e::UINT8){
                    std::string type_str = "uint8";
                    uint8_t val_from_adr = *reinterpret_cast<uint8_t*>(pm.adrMap[val_num]);
                    uint8_t val = pm.read<uint8_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %d, %d\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else if(val_type == Type_e::UINT16){
                    std::string type_str = "uint16";
                    uint16_t val_from_adr = *reinterpret_cast<uint16_t*>(pm.adrMap[val_num]);
                    uint16_t val = pm.read<uint16_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %d, %d\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else if(val_type == Type_e::UINT32){
                    std::string type_str = "uint32";
                    uint32_t val_from_adr = *reinterpret_cast<uint32_t*>(pm.adrMap[val_num]);
                    uint32_t val = pm.read<uint32_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %d, %d\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else if(val_type == Type_e::INT8){
                    std::string type_str = "int8";
                    int8_t val_from_adr = *reinterpret_cast<int8_t*>(pm.adrMap[val_num]);
                    int8_t val = pm.read<int8_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %d, %d\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else if(val_type == Type_e::INT16){
                    std::string type_str = "int16";
                    int16_t val_from_adr = *reinterpret_cast<int16_t*>(pm.adrMap[val_num]);
                    int16_t val = pm.read<int16_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %d\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else if(val_type == Type_e::INT32){
                    std::string type_str = "int32";
                    int32_t val_from_adr = *reinterpret_cast<int32_t*>(pm.adrMap[val_num]);
                    int32_t val = pm.read<int32_t>(val_num);
                    PRINTF_ASYNC("    %3d , %-7s, %-28s, %d\n", val_num, type_str.c_str(), key_str.c_str(), val, val_from_adr);
                }
                else{
                    PRINTF_ASYNC("  invalid type!\n");                
                }            

                hal::waitmsec(1);
            }
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "ttl") == 0){
            PRINTF_ASYNC("\n");                           
            PRINTF_ASYNC(";-------- teraterm macro file --------\n");                           
            auto &pm = ParameterManager::getInstance();
            for(int i=0;i<pm.valStrkeyMap.size();i++){

                uint16_t val_num = i;
                std::string key_str = pm.valStrkeyMap[val_num];
                Type_e val_type = pm.typeMap[val_num];
                
                if(key_str.empty()){
                    PRINTF_ASYNC("; %3d |\n", val_num);
                }

                else if(val_type == Type_e::FLOAT){
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

        if (ntlibc_strcmp(argv[1], "read") == 0 ) {
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            std::string param_name_str(argv[2]);
            auto &pm = ParameterManager::getInstance();
            if (pm.strkeyMap.find(param_name_str) == pm.strkeyMap.end()) {
                PRINTF_ASYNC("  %s | parameter not found!\n", param_name_str.c_str());
            }
            else{
                uint16_t val_num = pm.strkeyMap[param_name_str];
                Type_e val_type = pm.typeMap[val_num];

                if(val_type == Type_e::FLOAT){
                    float param_val = pm.read<float>(val_num);
                    std::string type_str = "float";                    
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %f\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::UINT8){
                    uint8_t param_val = pm.read<uint8_t>(val_num);
                    std::string type_str = "uint8_t";
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::UINT16){
                    uint16_t param_val = pm.read<uint16_t>(val_num);
                    std::string type_str = "uint16_t";
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::UINT32){
                    uint32_t param_val = pm.read<uint32_t>(val_num);
                    std::string type_str = "uint32_t";
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::INT8){
                    int8_t param_val = pm.read<int8_t>(val_num);
                    std::string type_str = "int8_t";
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::INT16){
                    int16_t param_val = pm.read<int16_t>(val_num);
                    std::string type_str = "int16_t";                    
                    PRINTF_ASYNC("    %3d , %-7s, %-20s, %d\n", val_num, type_str.c_str(), param_name_str.c_str(), param_val);
                }
                else if(val_type == Type_e::INT32){
                    int32_t param_val = pm.read<int32_t>(val_num);
                    std::string type_str = "int32_t";
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

