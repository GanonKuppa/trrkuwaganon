#include "imuDriver.h"

#include <math.h>
#include <string>

// Lib
#include "ntlibc.h"
#include "debugLog.h"

// Hal
#include "hal_spi.h"
#include "hal_timer.h"

// Msg
#include "msgBroker.h"
#include "imuMsg.h"

// Module
#include "parameterManager.h"


namespace module {

    static int16_t concatenate2Byte_int(uint8_t H_byte, uint8_t L_byte) {
        int32_t con;
        con = L_byte + (H_byte<<8);
        if (con > 32767) {
            con -=  65536;
        }
        return (int16_t)con;
    }

    ImuDriver::ImuDriver() :
    _ang_v_raw{0, 0, 0},
    _acc_raw{0, 0, 0},
    _ang_v_c{0.0f, 0.0f, 0.0f},
    _acc_c{0, 0, 0},
    _ang_v_f{0.0f, 0.0f, 0.0f},
    _acc_f{0.0f, 0.0f, 0.0f},
    _acc_norm(0.0f),
    _ang_v_f_int{0.0f, 0.0f, 0.0f},
    _temp_raw(0),
    _temp{0.0f},
    _imu_name("asm330lhh")
    {        
        setModuleName("ImuDriver");
        
        _writeReg(0x10, 0b10101010); // 加速度 6667Hz 4g LPF2_XL_EN = ON
        _writeReg(0x11, 0b10101100); // ジャイロ 6667Hz 2000dps
        _writeReg(0x13, 0b00000010); // ジャイロLPF ON
        _writeReg(0x15, 0b00000001); // ジャイロLPF 223Hz
        _writeReg(0x17, 0b01000000); // 加速度LPF 6667/20                

        ParameterManager& pm = ParameterManager::getInstance();
        _ang_v_offset[0] = pm.gyro0_x_offset;
        _ang_v_offset[1] = pm.gyro0_y_offset;        
        _ang_v_offset[2] = pm.gyro0_z_offset;

        _acc_offset[0] = 0.0f;
        _acc_offset[1] = 0.0f;
        _acc_offset[2] = 0.0f;
    }

    void ImuDriver::calibrateGyro(uint16_t num){
        _ang_v_f_int[0] = 0.0f;
        _ang_v_f_int[1] = 0.0f;
        _ang_v_f_int[2] = 0.0f;
        
        int8_t gyro_x[num];
        int8_t gyro_y[num];
        int8_t gyro_z[num];

        float gyro_x_sum = 0.0f;
        float gyro_y_sum = 0.0f;
        float gyro_z_sum = 0.0f;

        float offset_x = 0.0f;
        float offset_y = 0.0f;
        float offset_z = 0.0f;

        for (int i = 0; i < 3; i++) {
            _ang_v_offset[i] = 0;
        }

        for (uint16_t i = 0; i < num; i++) {
            gyro_x[i] = _ang_v_raw[0];
            gyro_y[i] = _ang_v_raw[1];
            gyro_z[i] = _ang_v_raw[2];
            hal::waitmsec(1);
            PRINTF_ASYNC("  %3d| calibrating... %d, %d, %d\n", i, gyro_x[i], gyro_y[i], gyro_z[i]);            
        }

        for (uint32_t i = 0; i < num; i++) {
            gyro_x_sum += (float) (gyro_x[i]);
            gyro_y_sum += (float) (gyro_y[i]);
            gyro_z_sum += (float) (gyro_z[i]);            
        }

        offset_x = gyro_x_sum / (float)num;
        offset_y = gyro_y_sum / (float)num;
        offset_z = gyro_z_sum / (float)num;

        PRINTF_ASYNC("gyro0 offset %f, %f, %f\n", offset_x, offset_y, offset_z);

        ParameterManager& pm = ParameterManager::getInstance();

        _ang_v_offset[0] = offset_x;
        _ang_v_offset[1] = offset_y;
        _ang_v_offset[2] = offset_z;

        pm.write<float>("gyro0_x_offset", _ang_v_offset[0]);
        pm.write<float>("gyro0_y_offset", _ang_v_offset[1]);
        pm.write<float>("gyro0_z_offset", _ang_v_offset[2]);
    }

    void ImuDriver::evalGyro(uint32_t num){
        float time = 0.0f;

        PRINTF_ASYNC("---------------------\n");
        PRINTF_ASYNC("time[sec], gyro_x[deg/s], gyro_y[deg/s], gyro_z[deg/s], gyro_x_int[deg], gyro_y_int[deg], gyro_z_int[deg]\n");
        _ang_v_f_int[0] = 0.0f;
        _ang_v_f_int[1] = 0.0f;
        _ang_v_f_int[2] = 0.0f;

        for (uint16_t i = 0; i < num; i++) {            
            uint32_t start_usec = hal::getElapsedUsec();
            PRINTF_ASYNC("%.3f, %f, %f, %f, %f, %f, %f\n", time, _ang_v_f[0], _ang_v_f[1], _ang_v_f[2], _ang_v_f_int[0], _ang_v_f_int[1], _ang_v_f_int[2]);
            time += 0.1;
            while(1){
                uint32_t end_usec = hal::getElapsedUsec();
                if(end_usec - start_usec > 100000) break;
            }
        }
    }



    void ImuDriver::update0(){

        const float gyro_scale = 0.07f; //-2000dps to 2000dps
        const float acc_scale = 0.0011964113f; //-4g to 4g //0.122 * 0.001 * 9.80665
        const float temp_scale = 0.00390625f;   // 256 LSB/degC
        const float temp_offset = 25.0f;

/*
        uint8_t out_temp_l = _readReg(0x20);
        uint8_t out_temp_h = _readReg(0x21);

        uint8_t outx_l_g = _readReg(0x22);
        uint8_t outx_h_g = _readReg(0x23);
        uint8_t outy_l_g = _readReg(0x24);
        uint8_t outy_h_g = _readReg(0x25);
        uint8_t outz_l_g = _readReg(0x26);
        uint8_t outz_h_g = _readReg(0x27);

        uint8_t outx_l_a = _readReg(0x28);
        uint8_t outx_h_a = _readReg(0x29);
        uint8_t outy_l_a = _readReg(0x2a);
        uint8_t outy_h_a = _readReg(0x2b);
        uint8_t outz_l_a = _readReg(0x2c);
        uint8_t outz_h_a = _readReg(0x2d);
*/
        uint8_t recv[15] = {0};
        _readRegBurst(recv);
        uint8_t out_temp_l = recv[1];
        uint8_t out_temp_h = recv[2];

        uint8_t outx_l_g = recv[3];
        uint8_t outx_h_g = recv[4];
        uint8_t outy_l_g = recv[5];
        uint8_t outy_h_g = recv[6];
        uint8_t outz_l_g = recv[7];
        uint8_t outz_h_g = recv[8];

        uint8_t outx_l_a = recv[9];
        uint8_t outx_h_a = recv[10];
        uint8_t outy_l_a = recv[11];
        uint8_t outy_h_a = recv[12];
        uint8_t outz_l_a = recv[13];
        uint8_t outz_h_a = recv[14];
        
        _ang_v_raw[0] = - concatenate2Byte_int(outx_h_g, outx_l_g);
        _ang_v_raw[1] = - concatenate2Byte_int(outy_h_g, outy_l_g);
        _ang_v_raw[2] = concatenate2Byte_int(outz_h_g, outz_l_g);

        _ang_v_c[0] = float(_ang_v_raw[0]) - _ang_v_offset[0];
        _ang_v_c[1] = float(_ang_v_raw[1]) - _ang_v_offset[1];
        _ang_v_c[2] = float(_ang_v_raw[2]) - _ang_v_offset[2];

        _ang_v_f[0] = float(_ang_v_c[0]) * gyro_scale;
        _ang_v_f[1] = float(_ang_v_c[1]) * gyro_scale;
        _ang_v_f[2] = float(_ang_v_c[2]) * gyro_scale;

        _acc_raw[0] = - concatenate2Byte_int(outx_h_a, outx_l_a);
        _acc_raw[1] = - concatenate2Byte_int(outy_h_a, outy_l_a);
        _acc_raw[2] = concatenate2Byte_int(outz_h_a, outz_l_a);

        _acc_c[0] = float(_acc_raw[0]) - _acc_offset[0];
        _acc_c[1] = float(_acc_raw[1]) - _acc_offset[1];
        _acc_c[2] = float(_acc_raw[2]) - _acc_offset[2];

        _acc_f[0] = float(_acc_c[0]) * acc_scale;
        _acc_f[1] = float(_acc_c[1]) * acc_scale;
        _acc_f[2] = float(_acc_c[2]) * acc_scale;

        _acc_norm = sqrtf(_acc_f[0] * _acc_f[0] + _acc_f[1] * _acc_f[1] + _acc_f[2] * _acc_f[2]) / 9.80665f;

        _temp_raw = concatenate2Byte_int(out_temp_h, out_temp_l);
        _temp = float(_temp_raw) * temp_scale + temp_offset;

        _ang_v_f_int[0] += _ang_v_f[0] * _delta_t;
        _ang_v_f_int[1] += _ang_v_f[1] * _delta_t;
        _ang_v_f_int[2] += _ang_v_f[2] * _delta_t;

        _publish();
    };

    void ImuDriver::debug(){          
        PRINTF_ASYNC("  ----------- %s -----------\n",_imu_name.c_str());
        PRINTF_ASYNC("  acc raw      : %d %d %d\n", _acc_raw[0], _acc_raw[1], _acc_raw[2]);
        PRINTF_ASYNC("  acc offset   : %f %f %f\n", _acc_offset[0], _acc_offset[1], _acc_offset[2]);
        PRINTF_ASYNC("  acc_c        : %f %f %f\n", _acc_c[0], _acc_c[1], _acc_c[2]);
        PRINTF_ASYNC("  acc  [m/s^2] : %f %f %f\n", _acc_f[0], _acc_f[1], _acc_f[2]);
        PRINTF_ASYNC("  acc norm [G] : %f\n", _acc_norm);
        
        PRINTF_ASYNC("  ------------------------\n");
        PRINTF_ASYNC("  gyro raw     : %d %d %d\n", _ang_v_raw[0], _ang_v_raw[1], _ang_v_raw[2]);
        PRINTF_ASYNC("  gyro offset  : %f %f %f\n", _ang_v_offset[0], _ang_v_offset[1], _ang_v_offset[2]);        
        PRINTF_ASYNC("  gyro_c       : %f %f %f\n", _ang_v_c[0], _ang_v_c[1], _ang_v_c[2]);
        PRINTF_ASYNC("  gyro [deg/s] : %f %f %f\n", _ang_v_f[0], _ang_v_f[1], _ang_v_f[2]);
        PRINTF_ASYNC("  ------------------------\n");
        PRINTF_ASYNC("  temp raw     : %d\n", _temp_raw);
        PRINTF_ASYNC("  temp [degC]  : %f\n", _temp);
    }

    void ImuDriver::whoAmI(){
        uint16_t adress = 0x0F;

        uint8_t val = _readReg(adress);
        PRINTF_ASYNC(" %s who am i : %x\n", _imu_name.c_str(), val); 
    }


    void ImuDriver::_writeReg(uint8_t adress, uint8_t data) {
        hal::useCS0SPI0();
        hal::setEnableSPI0(1);
        uint8_t sendBuf[2];
        uint8_t recvBuf[2];

        sendBuf[0] = adress;
        sendBuf[1] = data;
        hal::communicateNbyteSPI0(sendBuf, recvBuf, 2);
    }

    uint8_t ImuDriver::_readReg(uint8_t adress) {
        hal::useCS0SPI0();
        hal::setEnableSPI0(1);

        uint8_t sendBuf[2];
        uint8_t recvBuf[2];

        const uint8_t READ_FLAG = 0x80;
        sendBuf[0] = READ_FLAG | adress;
        sendBuf[1] = 0x00;
        hal::communicateNbyteSPI0(sendBuf, recvBuf, 2);
        return recvBuf[1];
    }

    void ImuDriver::_readRegBurst(uint8_t* recv){
        hal::useCS0SPI0();
        hal::setEnableSPI0(1);

        uint8_t send[15] = {0};
        const uint8_t READ_FLAG = 0x80;
        const uint8_t adress = 0x20;        
        send[0] = READ_FLAG | adress;
        hal::communicateNbyteSPI0(send, recv, 15);

    }

    void ImuDriver::_publish(){
        const float DEG2RAD = 57.2957795131f;
        ImuMsg msg;
        msg.pitchrate = _ang_v_f[0] * DEG2RAD;
        msg.rollrate = _ang_v_f[1] * DEG2RAD;
        msg.yawrate = _ang_v_f[2] * DEG2RAD;
        
        msg.acc_x = _acc_f[0];
        msg.acc_y = _acc_f[1];
        msg.acc_z = _acc_f[2];

        msg.temp = _temp;
        msg.stop_time = 0.0f;
        msg.upside_down_time = 0.0f;
        msg.is_stop = 0.0f;
        msg.is_upside_down = 0.0f;
        publishMsg(msg_id::IMU, &msg);
    }



    int usrcmd_imuDriver(int argc, char **argv){

        if (ntlibc_strcmp(argv[1], "status") == 0) {
            ImuDriver::getInstance().debug();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "whoami") == 0) {
            ImuDriver::getInstance().whoAmI();
            return 0;
        }

        if (ntlibc_strcmp(argv[1], "calib") == 0){
            if(argc != 3){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            if(ntlibc_strcmp(argv[2], "gyro") == 0){
                ImuDriver::getInstance().calibrateGyro(1000);
                return 0;
            }
        }

        if (ntlibc_strcmp(argv[1], "eval") == 0){
            if(argc != 4){
                PRINTF_ASYNC("  invalid param num!\n");
                return -1;
            }
            if(ntlibc_strcmp(argv[2], "gyro") == 0){
                std::string num_str(argv[3]);
                int32_t num = std::stoi(num_str);
                ImuDriver::getInstance().evalGyro(num);
                return 0;
            }
        }


        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;

    };
}
