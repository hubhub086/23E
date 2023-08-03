cmake_minimum_required(VERSION 3.23.0-rc4)
project(23E)

set(CMAKE_CXX_STANDARD 11)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)#寻找OpenCV所需要的包
find_package(JetsonGPIO)
include_directories(${OpenCV_INCLUDE_DIRS})#引用OpenCV的include

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif ("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else ()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif ()

add_executable(23E main.cpp uart.cpp tft.cpp)
target_link_libraries(23E ${OpenCV_LIBS} ${WIRINGPI_LIBRARIES} Threads::Threads)#链接OpenCV库
target_link_libraries(23E JetsonGPIO::JetsonGPIO)
target_link_libraries(23E /usr/local/lib/libspidev-lib++.a)