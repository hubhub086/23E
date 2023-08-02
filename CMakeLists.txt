cmake_minimum_required(VERSION 3.23.0-rc4)
project(23E)

set(CMAKE_CXX_STANDARD 11)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)#寻找OpenCV所需要的包
find_package(JetsonGPIO)
include_directories(${OpenCV_INCLUDE_DIRS})#引用OpenCV的include

add_executable(23E main.cpp uart.cpp tft.cpp)
target_link_libraries(23E ${OpenCV_LIBS} ${WIRINGPI_LIBRARIES} Threads::Threads)#链接OpenCV库
target_link_libraries(23E JetsonGPIO::JetsonGPIO)
target_link_libraries(23E /usr/local/lib/libspidev-lib++.a)