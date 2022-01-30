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
        void registration(uint16_t val_num, T& r_val, std::string key, T default_val);

        //----管理下においた変数たち-----
        float mass; //0
        float dia_tire; //1
        float tread; //2
        float duty_limit; //3
        uint8_t silent_flag; //4
        float test_run_v; //5
        float test_run_a; //6
        float test_run_x; //7
        uint8_t test_run_wall_flag; //8
        float v_search_run; //9
        float a_search_run; //10
        float spin_yawrate_max; //11
        float spin_yawacc; //12
        uint8_t goal_x; //13
        uint8_t goal_y; //14
        float search_limit_time_sec; //15
        uint8_t logger_skip_mod; //16
        float wall2mouse_center_dist; //17
        float suction_duty_search; //18
        float suction_duty_shortest; //19
        uint8_t corner_correction_enable; //20
        uint8_t turn_pre_corner_correction_enable; //21

        float duty_coef_left_p; //30
        float duty_offset_left_p; //31
        float duty_coef_right_p; //32
        float duty_offset_right_p; //33
        float duty_coef_left_m; //34
        float duty_offset_left_m; //35
        float duty_coef_right_m; //36
        float duty_offset_right_m; //37
 

        uint8_t v_fb_enable; //50
        uint8_t v_ff_enable; //51
        uint8_t v_saturation_enable; //52
        uint8_t v_i_saturation_enable; //53
        
        uint8_t yawrate_fb_enable; //54
        uint8_t yawrate_ff_enable; //55
        uint8_t yawrate_saturation_enable; //56
        uint8_t yawrate_i_saturation_enable; //57
        
        uint8_t yaw_fb_enable; //58
        uint8_t yaw_saturation_enable; //59
        uint8_t yaw_i_saturation_enable; //60

        uint8_t wall_fb_enable; //61
        uint8_t wall_saturation_enable; //62
        uint8_t wall_i_saturation_enable; //63

        uint8_t wall_diag_fb_enable; //64
        uint8_t wall_diag_saturation_enable; //65
        uint8_t wall_diag_i_saturation_enable; //66
        
        float diag_ctrl_dist_thr_r; //67
        float diag_ctrl_dist_thr_l;
      
        
        float v_ff_coef; //80
        float v_ff_offset; //81
        float a_ff_coef; //82
        float a_ff_offset; //83
 
        float yawrate_ff_coef; //84
        float yawrate_ff_offset; //85
        float yawacc_ff_coef; //86
        float yawacc_ff_offset; //87


        float search_v_p; //88
        float search_v_i; //89
        float search_v_d; //90
        float search_v_f; //91
        float search_yawrate_p; //92
        float search_yawrate_i; //93
        float search_yawrate_d; //94
        float search_yawrate_f; //95
        float search_yaw_p; //96
        float search_yaw_i; //97   

        float fast_v_p; //98
        float fast_v_i; //99
        float fast_v_d; //100
        float fast_v_f; //101
        float fast_yawrate_p; //102
        float fast_yawrate_i; //103
        float fast_yawrate_d; //104
        float fast_yawrate_f; //105
        float fast_yaw_p; //106
        float fast_yaw_i; //107  

        float spin_v_p; //108
        float spin_v_i; //109
        float spin_v_d; //110
        float spin_v_f; //111  
        float spin_yawrate_p; //112
        float spin_yawrate_i; //113
        float spin_yawrate_d; //114
        float spin_yawrate_f; //115
        float spin_yaw_p; //116
        float spin_yaw_i; //117

        float wall_p; //118
        float wall_i; //119
        float wall_d; //120
        float wall_f; //121
        float wall_diag_p; //122
        float wall_diag_i; //123
        float wall_diag_d; //124
        float wall_diag_f; //125

        
        float v_saturation_offset_duty; //126
        float v_saturation_ff_multiplier; //127
         
        float yawrate_saturation_offset_duty; //128
        float yawrate_saturation_ff_multiplier; //129
                
        float yaw_saturation; //130
        float wall_saturation; //131
        float wall_diag_saturation; //132

        float v_i_saturation; //133
        float yawrate_i_saturation; //134
        float yaw_i_saturation; //135
        float wall_i_saturation; //136
        float wall_diag_i_saturation; //137

        float wall_corner_read_offset_r; //138
        float wall_corner_read_offset_l; //139
        uint16_t wall_corner_threshold_on_r; //140
        uint16_t wall_corner_threshold_off_r; //141
        uint16_t wall_corner_threshold_on_l; //142
        uint16_t wall_corner_threshold_off_l; //143

        float diag_r_corner_read_offset; //144
        float diag_l_corner_read_offset; //145
        uint16_t diag_corner_threshold_on_r; //146
        uint16_t diag_corner_threshold_off_r; //147
        uint16_t diag_corner_threshold_on_l; //148
        uint16_t diag_corner_threshold_off_l; //149

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

        float dial_p; //185
        float dial_i; //186
        float dial_i_limit; //187
        float dial_limit; //188
        float cp_coef; //189        
        float wall_dist_p; //190
        float wall_dist_i; //191
        float wall_diff_p; //192
        float wall_diff_i; //193
        float wall_al_thr; //194
        float wall_l_thr; //195
        float wall_r_thr; //196
        float wall_ar_thr; //197
        float wall_center_l; //198
        float wall_center_r; //199

        float shortest_0_v; //200
        float shortest_0_v_d; //201
        float shortest_0_v_90; //202
        float shortest_0_v_l90; //203
        float shortest_0_v_180; //204
        float shortest_0_v_d90; //205
        float shortest_0_v_45; //206
        float shortest_0_v_135; //207
        float shortest_0_a; //208
        float shortest_0_a_diag; //209
        ; //210
        ; //211
        ; //212
        ; //213
        ; //214
        ; //215
        ; //216
        ; //217
        ; //218
        ; //219
        float shortest_1_v; //220
        float shortest_1_v_d; //221
        float shortest_1_v_90; //222
        float shortest_1_v_l90; //223
        float shortest_1_v_180; //224
        float shortest_1_v_d90; //225
        float shortest_1_v_45; //226
        float shortest_1_v_135; //227
        float shortest_1_a; //228
        float shortest_1_a_diag; //229
        ; //230
        ; //231
        ; //232
        ; //233
        ; //234
        ; //235
        ; //236
        ; //237
        ; //238
        ; //239
        float shortest_2_v; //240
        float shortest_2_v_d; //241
        float shortest_2_v_90; //242
        float shortest_2_v_l90; //243
        float shortest_2_v_180; //244
        float shortest_2_v_d90; //245
        float shortest_2_v_45; //246
        float shortest_2_v_135; //247
        float shortest_2_a; //248
        float shortest_2_a_diag; //249
        ; //250
        ; //251
        ; //252
        ; //253
        ; //254
        ; //255
        ; //256
        ; //257
        ; //258
        ; //259
        float shortest_3_v; //260
        float shortest_3_v_d; //261
        float shortest_3_v_90; //262
        float shortest_3_v_l90; //263
        float shortest_3_v_180; //264
        float shortest_3_v_d90; //265
        float shortest_3_v_45; //266
        float shortest_3_v_135; //267
        float shortest_3_a; //268
        float shortest_3_a_diag; //269
        ; //270
        ; //271
        ; //272
        ; //273
        ; //274
        ; //275
        ; //276
        ; //277
        ; //278
        ; //279
        float shortest_4_v; //280
        float shortest_4_v_d; //281
        float shortest_4_v_90; //282
        float shortest_4_v_l90; //283
        float shortest_4_v_180; //284
        float shortest_4_v_d90; //285
        float shortest_4_v_45; //286
        float shortest_4_v_135; //287
        float shortest_4_a; //288
        float shortest_4_a_diag; //289

        

        //----管理下においた変数たち-----

      private:
        friend class BaseModule<ParameterManager>;
        ParameterManager();
    };

    int usrcmd_parameterManager(int argc, char **argv);

}
