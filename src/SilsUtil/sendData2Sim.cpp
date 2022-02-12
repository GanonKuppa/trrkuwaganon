#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>

#include "picojson.h"
#include "udp.h"

#include "sendData2Sim.h"

namespace sim {
    void initSimConnection() {
        char buf[256];
        // WSL2でホストのWindowsのIPアドレスを取得するための処理
        {
            FILE* fp;
            if ((fp = popen("cat /etc/resolv.conf | grep nameserver | awk '{print $2}'", "r")) == NULL) {
                fprintf(stderr, "パイプのオープンに失敗しました！: %s", "cat /etc/resolv.conf | grep nameserver | awk '{print $2}'");
                return;
            }

            printf("host ip:");
            while (fgets(buf, sizeof(buf), fp) != NULL) {
                printf("%s", buf);
            }                    
            pclose(fp);
        }
        initUdpClient(buf, 2020);
    }

    void finalizeSimConnection() {
        finalizeUdpClient();
    }

    void setRobotPos(float x, float y, float ang, float v) {
        picojson::object obj;

        // データの追加
        obj.emplace(std::make_pair("cmd", "SET_ROBOT_POS"));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        obj.emplace(std::make_pair("ang", ang));
        obj.emplace(std::make_pair("v", v));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        val.serialize();
        sendUdpString(val.serialize());
    }

    void setRobotColor(uint8_t r, uint8_t g, uint8_t b) {

        picojson::object obj;

        // データの追加
        obj.emplace(std::make_pair("cmd", "SET_ROBOT_COLOR"));
        obj.emplace(std::make_pair("r", (double)r));
        obj.emplace(std::make_pair("g", (double)g));
        obj.emplace(std::make_pair("b", (double)b));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        // return std::string
        val.serialize();
        sendUdpString(val.serialize());

    }


    void setTargetPos(float x, float y, float ang) {
        picojson::object obj;

        // データの追加
        obj.emplace(std::make_pair("type", "target_pos"));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        obj.emplace(std::make_pair("ang", ang));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        // return std::string
        val.serialize();
        sendUdpString( val.serialize());
    }

    void setWallsWithoutOuter32(uint32_t* walls_vertical, uint32_t* walls_horizontal) {
        picojson::object obj;
        obj.emplace(std::make_pair("cmd", "SET_WALLS_WITHOUT_OUTER_32"));

        std::stringstream vertical_ss;
        std::stringstream horizontal_ss;

        for(int i=0; i<31; i++) {
            uint32_t v_val = ((walls_vertical[i] & 0xFF000000) >> (8*3)) +
                             ((walls_vertical[i] & 0x00FF0000) >> (8*1)) +
                             ((walls_vertical[i] & 0x0000FF00) << (8*1)) +
                             ((walls_vertical[i] & 0x000000FF) << (8*3));

            vertical_ss  << std::setw(8) << std::setfill('0') << std::hex << v_val;

            uint32_t h_val = ((walls_horizontal[i] & 0xFF000000) >> (8*3)) +
                             ((walls_horizontal[i] & 0x00FF0000) >> (8*1)) +
                             ((walls_horizontal[i] & 0x0000FF00) << (8*1)) +
                             ((walls_horizontal[i] & 0x000000FF) << (8*3));
            horizontal_ss << std::setw(8) << std::setfill('0') << std::hex << h_val;
        }
        obj.emplace(std::make_pair("walls_v_hex", vertical_ss.str()));
        obj.emplace(std::make_pair("walls_h_hex", horizontal_ss.str()));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);        
        val.serialize();
        sendUdpString( val.serialize());
    }


    void setWallsWithoutOuter32(uint32_t* walls_vertical, uint32_t* walls_horizontal, uint32_t* transparent_v_mask, uint32_t* transparent_h_mask) {
        picojson::object obj;
        obj.emplace(std::make_pair("cmd", "SET_WALLS_WITHOUT_OUTER_32"));

        std::stringstream vertical_ss;
        std::stringstream horizontal_ss;
        std::stringstream transparent_v_mask_ss;
        std::stringstream transparent_h_mask_ss;

        for(int i=0; i<31; i++) {
            uint32_t v_val = ((walls_vertical[i] & 0xFF000000) >> (8*3)) +
                             ((walls_vertical[i] & 0x00FF0000) >> (8*1)) +
                             ((walls_vertical[i] & 0x0000FF00) << (8*1)) +
                             ((walls_vertical[i] & 0x000000FF) << (8*3));

            vertical_ss  << std::setw(8) << std::setfill('0') << std::hex << v_val;

            uint32_t h_val = ((walls_horizontal[i] & 0xFF000000) >> (8*3)) +
                             ((walls_horizontal[i] & 0x00FF0000) >> (8*1)) +
                             ((walls_horizontal[i] & 0x0000FF00) << (8*1)) +
                             ((walls_horizontal[i] & 0x000000FF) << (8*3));
            horizontal_ss << std::setw(8) << std::setfill('0') << std::hex << h_val;

            uint32_t t_v_val = ((transparent_v_mask[i] & 0xFF000000) >> (8*3)) +
                             ((transparent_v_mask[i] & 0x00FF0000) >> (8*1)) +
                             ((transparent_v_mask[i] & 0x0000FF00) << (8*1)) +
                             ((transparent_v_mask[i] & 0x000000FF) << (8*3));
            t_v_val = ~t_v_val;
            transparent_v_mask_ss  << std::setw(8) << std::setfill('0') << std::hex << t_v_val;

            uint32_t t_h_val = ((transparent_h_mask[i] & 0xFF000000) >> (8*3)) +
                             ((transparent_h_mask[i] & 0x00FF0000) >> (8*1)) +
                             ((transparent_h_mask[i] & 0x0000FF00) << (8*1)) +
                             ((transparent_h_mask[i] & 0x000000FF) << (8*3));
            t_h_val = ~t_h_val;
            transparent_h_mask_ss << std::setw(8) << std::setfill('0') << std::hex << t_h_val;

        }
        obj.emplace(std::make_pair("walls_v_hex", vertical_ss.str()));
        obj.emplace(std::make_pair("walls_h_hex", horizontal_ss.str()));
        obj.emplace(std::make_pair("transparent_v_mask_hex", transparent_v_mask_ss.str()));
        obj.emplace(std::make_pair("transparent_h_mask_hex", transparent_h_mask_ss.str()));

        // 文字列にするためにvalueを使用
        picojson::value val(obj);        
        val.serialize();
        sendUdpString(val.serialize());
    }


    void setNeedle(float x, float y) {
        picojson::object obj;

        // データの追加
        obj.emplace(std::make_pair("type", "needle"));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        // return std::string
        val.serialize();
        sendUdpString( val.serialize());
    }

    void setReload() {
        {
            picojson::object obj;
            obj.emplace(std::make_pair("cmd", "REMOVE_CONTRAIL"));
            picojson::value val(obj);
            val.serialize();
            sendUdpString( val.serialize());
        }

        {
            picojson::object obj;
            obj.emplace(std::make_pair("cmd", "REMOVE_ALL_NAMED_OBJECT"));
            picojson::value val(obj);
            val.serialize();
            sendUdpString( val.serialize());
        }
    }

    void setNumberSqure(float num, float x, float y) {
        picojson::object obj;

        // データの追加
        obj.emplace(std::make_pair("type", "number_squre"));
        obj.emplace(std::make_pair("num", num));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);        
        val.serialize();
        sendUdpString( val.serialize());

    }

    void updateDataView(float x, float y, float ang, float v){
        picojson::object obj;
        // データの追加
        obj.emplace(std::make_pair("cmd", "UPDATE_DATA_VIEW"));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        obj.emplace(std::make_pair("ang", ang));
        obj.emplace(std::make_pair("v", v));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        val.serialize();
        sendUdpString(val.serialize());
    }

    void addPointRobotContrail(float x, float y, float ang, float v){
        picojson::object obj;
        // データの追加
        obj.emplace(std::make_pair("cmd", "ADD_POINT_ROBOT_CONTRAIL"));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        obj.emplace(std::make_pair("ang", ang));
        obj.emplace(std::make_pair("v", v));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        val.serialize();
        sendUdpString(val.serialize());
    }
    
    void addPointTargetContrail(float x, float y, float ang, float v){
        picojson::object obj;
        // データの追加
        obj.emplace(std::make_pair("cmd", "ADD_POINT_TARGET_CONTRAIL"));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        obj.emplace(std::make_pair("ang", ang));
        obj.emplace(std::make_pair("v", v));
        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        val.serialize();
        sendUdpString(val.serialize());
    }

    void updateDataView(float time, float bat_vol, float l_motor_vol, float r_motor_vol, float x, float y, float yaw, float v){
        picojson::object obj;
        // データの追加
        obj.emplace(std::make_pair("cmd", "UPDATE_DATA_VIEW"));
        obj.emplace(std::make_pair("time", time));
        obj.emplace(std::make_pair("bat", bat_vol));
        obj.emplace(std::make_pair("V_l", l_motor_vol));
        obj.emplace(std::make_pair("V_r", r_motor_vol));
        obj.emplace(std::make_pair("x", x));
        obj.emplace(std::make_pair("y", y));
        obj.emplace(std::make_pair("ang", yaw));
        obj.emplace(std::make_pair("v", v));

        // 文字列にするためにvalueを使用
        picojson::value val(obj);
        val.serialize();
        sendUdpString(val.serialize());
    }

}