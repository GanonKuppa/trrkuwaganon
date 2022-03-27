cmake_minimum_required(VERSION 3.10)
project(test_traj)

#set(CMAKE_MAKE_PROGRAM "C:/Program Files (x86)/GnuWin32/bin/make.exe")
#message(cmake -G "MinGW Makefiles" ..) windowsでは以下でcmakeを叩くこと
#set(CMAKE_C_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/gcc")
#set(CMAKE_CXX_COMPILER "C:/Program Files/mingw-w64/x86_64-8.1.0-posix-sjlj-rt_v6-rev0/mingw64/bin/g++")

message(STATUS  "======== trrkuwaganon test_traj ========")                     

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O2 -Wall -fdiagnostics-color -DSILS")


message(STATUS ${APP_CPP})
message(STATUS ${APP_C})

# h
set(APP_LIB_H ../../src/ApplicationLayer/Lib)
set(APP_OBJECT_H ../../src/ApplicationLayer/Object)

include_directories(
   ${APP_LIB_H}
   ${APP_OBJECT_H}
)

add_executable(main 
    main.cpp
    ../../src/ApplicationLayer/Lib/mollifier.cpp
    ../../src/ApplicationLayer/Object/turnPreCalculation.cpp
)
