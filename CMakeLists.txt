cmake_minimum_required (VERSION 2.6)
project (PixyVision)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/")

find_package(LibUSB)

add_library(pixy2usb libs/pixy2/src/common/src/chirp.cpp libs/pixy2/src/host/libpixyusb2/src/libpixyusb2.cpp libs/pixy2/src/host/libpixyusb2/src/usblink.cpp libs/pixy2/src/host/libpixyusb2/src/util.cpp)
target_include_directories(pixy2usb PUBLIC libs/pixy2/src/common/inc libs/pixy2/src/host/libpixyusb2/include libs/pixy2/src/host/arduino/libraries/Pixy2 ${LibUSB_INCLUDE_DIRS})

add_executable(Vision main.cpp)
target_include_directories(Vision PUBLIC libs/pixy2/src/host/libpixyusb2/include libs/gcem/include)
target_link_libraries(Vision pixy2usb ${LibUSB_LIBRARIES})
