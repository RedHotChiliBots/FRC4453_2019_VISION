cmake_minimum_required (VERSION 2.6)
project (PixyVision)
set (CMAKE_CXX_STANDARD 17)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(LibUSB)

set(WITHOUT_JAVA ON CACHE BOOl "don't include java and JNI in the build" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "build with shared libs (needed for JNI)" FORCE)
set(WITHOUT_CSCORE ON CACHE BOOL "Don't build cscore (removes OpenCV requirement)" FORCE)
set(WITHOUT_ALLWPILIB ON CACHE BOOL "Don't build allwpilib (removes OpenCV requirement)" FORCE)

add_subdirectory(libs/allwpilib)

add_library(pixy2usb libs/pixy2/src/common/src/chirp.cpp libs/pixy2/src/host/libpixyusb2/src/libpixyusb2.cpp libs/pixy2/src/host/libpixyusb2/src/usblink.cpp libs/pixy2/src/host/libpixyusb2/src/util.cpp)
target_include_directories(pixy2usb PUBLIC libs/pixy2/src/common/inc libs/pixy2/src/host/libpixyusb2/include libs/pixy2/src/host/arduino/libraries/Pixy2 ${LibUSB_INCLUDE_DIRS})

add_executable(Vision main.cpp geom.hpp)
target_include_directories(Vision PUBLIC libs/pixy2/src/host/libpixyusb2/include libs/gcem/include libs/allwpilib/ntcore/src/main/native/include)
target_link_libraries(Vision pixy2usb ${LibUSB_LIBRARIES} ntcore)
