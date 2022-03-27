cmake_minimum_required(VERSION 3.10)
project(test_trajSetpoint)

#set(CMAKE_MAKE_PROGRAM "C:/Program Files (x86)/GnuWin32/bin/make.exe")
# message(cmake -G "MinGW Makefiles" ..) windowsでは以下でcmakeを叩くこと
#set(CMAKE_C_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/gcc")
#set(CMAKE_CXX_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/g++")

message(STATUS  "======== trrkuwaganon test_trajSetpoint ========")                     

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-O2 -Wall -fdiagnostics-color -DSILS")

# cpp c
file(GLOB_RECURSE APP_CPP ../../src/ApplicationLayer/*.cpp)
file(GLOB_RECURSE APP_C ../../src/ApplicationLayer/*.c)
file(GLOB_RECURSE HAL_CPP ../../src/HardwareAbstractionLayer/*.cpp)
file(GLOB_RECURSE HAL_C ../../src/HardwareAbstractionLayer/*.c)
file(GLOB_RECURSE SILS_UTIL_CPP ../../src/SilsUtil/*.cpp)
file(GLOB_RECURSE SILS_UTIL_C ../../src/SilsUtil/*.c)

message(STATUS ${APP_CPP})
message(STATUS ${APP_C})
message(STATUS ${HAL_CPP})
message(STATUS ${HAL_C})
message(STATUS ${SILS_UTIL_CPP})
message(STATUS ${SILS_UTIL_C})


# h
set(SRC_DIR ../../src)
set(APP_ACTIVITY_H ../../src/ApplicationLayer/Activity)
set(APP_LIB_H ../../src/ApplicationLayer/Lib)
set(APP_MESSAGE_H ../../src/ApplicationLayer/Message/)
set(APP_MODULE_H ../../src/ApplicationLayer/Module)
set(APP_OBJECT_H ../../src/ApplicationLayer/Object)
set(HAL_H ../../src/HardwareAbstractionLayer)
set(SILS_UTIL_H ../../src/SilsUtil)

set(EIGEN_DIR ../../src/ApplicationLayer/Lib/Eigen)
set(NTSHELL_CORE_DIR ../../src/ApplicationLayer/Lib/ntshell/core)
set(NTSHELL_UTIL_DIR ../../src/ApplicationLayer/Lib/ntshell/util)

include_directories(
   ${SRC_DIR}
   ${APP_ACTIVITY_H}
   ${APP_LIB_H}
   ${APP_MESSAGE_H}
   ${APP_MODULE_H}
   ${APP_OBJECT_H}
   ${HAL_H}
#   ${SILS_UTIL_H}
   ${EIGEN_DIR}
   ${NTSHELL_CORE_DIR}
   ${NTSHELL_UTIL_DIR}
)

add_executable(main 
    main.cpp
    ../../src/oneshotTask.cpp
    ${APP_CPP}
    ${APP_C}
    ${HAL_CPP}
    ${HAL_C}
#    ${SILS_UTIL_CPP}
#    ${SILS_UTIL_C}
)

#target_link_libraries(
#    main 
#    wsock32 
#    ws2_32 
#    Xinput
#    winmm
#)