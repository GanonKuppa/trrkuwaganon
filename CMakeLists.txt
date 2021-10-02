cmake_minimum_required(VERSION 3.1)

# message(cmake -G "MinGW Makefiles" ..) windowsでは以下でcmakeを叩くこと

message(STATUS  "======== scfkuwaganon sils cmake ========")                     
set(CMAKE_C_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/gcc")
set(CMAKE_CXX_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/g++")
set(CMAKE_CXX_STANDARD, 17)
set(CMAKE_CXX_FLAGS "-O2 -Wall -DSILS")

set(APP_ACTIVITY_H src/ApplicationLayer/Activity)
file(GLOB_RECURSE APP_ACTIVITY_CPP src/ApplicationLayer/Activity/*.cpp)

set(HAL_H src/HardwareAbstractionLayer)
file(GLOB_RECURSE HAL_CPP src/HardwareAbstractionLayer/*.cpp)

set(SILS_UTIL_H src/SilsUtil)
file(GLOB_RECURSE SILS_UTIL_CPP src/SilsUtil/*.cpp)

set(APP_MODULE_H src/ApplicationLayer/Module)
file(GLOB_RECURSE MODULE_CPP src/ApplicationLayer/Module/*.cpp)

set(APP_MESSAGE_H src/ApplicationLayer/Message/)
file(GLOB_RECURSE APP_MESSAGE_CPP src/ApplicationLayer/Message/*.cpp)

set(EIGEN_DIR src/ApplicationLayer/Lib/Eigen)
set(LIB_H src/ApplicationLayer/Lib)


message(STATUS ${HAL_H})
message(STATUS ${APP_ACTIVITY_H})
message(STATUS ${APP_MODULE_H})
message(STATUS ${APP_MESSAGE_H})
message(STATUS ${SILS_UTIL_H})


include_directories(
    ${EIGEN_DIR}
    ${HAL_H}
    ${APP_ACTIVITY_H}
    ${APP_MODULE_H}
    ${APP_MESSAGE_H}
    ${SILS_UTIL_H}
    ${LIB_H}
)

#target_link_libraries(main -lboost_system -lboost_filesystem)

add_executable(sils_main 
    src/sils_main.cpp
    ${APP_ACTIVITY_CPP}
    ${HAL_CPP}
    ${MODULE_CPP}
    ${SILS_UTIL_CPP}
)
target_link_libraries(sils_main 
    wsock32 
    ws2_32 
    Xinput
    winmm
)
