#include "wheelOdometry.h"

#include "parameterManager.h"
#include "hal_spi.h"

#include "ntlibc.h"

namespace module {
    WheelOdometry::WheelOdometry() :
    _count_r(0),
    _count_l(0),
    _ang_r(0.0f),
    _ang_l(0.0f),
    _v_r(0.0f),
    _v_l(0.0f),
    _v(0.0f),
    _v_ave(0.0f)

    {
        setModuleName("WheelOdometry");
        //ParameterManager& pm = ParameterManager::getInstance();
    }





    void WheelOdometry::update0(){
        hal::useCS0SPI1();
        _count_r = hal::communicate16bitSPI1(0);
        _ang_r = (float)_count_r / ENC_RES * 360.0f;

        hal::useCS1SPI1();
        _count_l = hal::communicate16bitSPI1(0);
        _ang_l = (float)_count_l / ENC_RES * 360.0f;
    }

    void WheelOdometry::debug(){
        PRINTF_ASYNC("  ang_r : %f\n", _ang_r);
        PRINTF_ASYNC("  ang_l : %f\n", _ang_l);
        PRINTF_ASYNC("  count_r : %d\n", _count_r);
        PRINTF_ASYNC("  count_l : %d\n", _count_l);
    }

    int usrcmd_wheelOdometry(int argc, char **argv){
        if (ntlibc_strcmp(argv[1], "status") == 0) {
            WheelOdometry::getInstance().debug();
            return 0;
        }
        PRINTF_ASYNC("  Unknown sub command found\r\n");
        return -1;

    };


}
