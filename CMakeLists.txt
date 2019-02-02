cmake_minimum_required (VERSION 2.6)
project (PixyVision)

# Options for wpilib.
set(WITHOUT_JAVA ON CACHE BOOL "don't include java and JNI in the build" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "build with shared libs (needed for JNI)" FORCE)
set(WITHOUT_CSCORE ON CACHE BOOL "Don't build cscore (removes OpenCV requirement)" FORCE)
set(WITHOUT_ALLWPILIB ON CACHE BOOL "Don't build allwpilib (removes OpenCV requirement)" FORCE)

# Add wpilib.
add_subdirectory(libs/allwpilib)

# Get our target triple (needed to configure libusb).
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine OUTPUT_VARIABLE GCC_TARGET_TRIPLE)
string(STRIP ${GCC_TARGET_TRIPLE} GCC_TARGET_TRIPLE)
message(STATUS "Target triple: ${GCC_TARGET_TRIPLE}")

# Add libusb project.
include(ExternalProject)
ExternalProject_Add(
  libusb
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/libusb-prefix
  UPDATE_COMMAND ./bootstrap.sh
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs/libusb
  INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/libusb
  CONFIGURE_COMMAND CFLAGS=${CMAKE_C_FLAGS} CXXFLAGS=${CMAKE_CXX_FLAGS} ${CMAKE_CURRENT_SOURCE_DIR}/libs/libusb/configure --prefix=<INSTALL_DIR> --disable-shared --disable-udev --host=${GCC_TARGET_TRIPLE}
  BUILD_COMMAND make clean && CFLAGS=${CMAKE_C_FLAGS} CXXFLAGS=${CMAKE_CXX_FLAGS} make
)

# Pixy library
add_library(pixy2usb libs/pixy2/src/common/src/chirp.cpp libs/pixy2/src/host/libpixyusb2/src/libpixyusb2.cpp libs/pixy2/src/host/libpixyusb2/src/usblink.cpp libs/pixy2/src/host/libpixyusb2/src/util.cpp)
add_dependencies(pixy2usb libusb)
target_include_directories(pixy2usb PUBLIC libs/pixy2/src/common/inc libs/pixy2/src/host/libpixyusb2/include libs/pixy2/src/host/arduino/libraries/Pixy2 ${CMAKE_CURRENT_BINARY_DIR}/libusb/include/libusb-1.0)
target_link_libraries(pixy2usb ${CMAKE_CURRENT_BINARY_DIR}/libusb/lib/libusb-1.0.a)

# Main binary
add_executable(Vision main.cpp geom.hpp cammath.hpp)
target_include_directories(Vision PUBLIC libs/pixy2/src/host/libpixyusb2/include libs/gcem/include libs/allwpilib/ntcore/src/main/native/include)
target_link_libraries(Vision pixy2usb ${LibUSB_LIBRARIES} ntcore)
set_property(TARGET Vision PROPERTY CXX_STANDARD 17)
set_property(TARGET Vision PROPERTY CXX_STANDARD_REQUIRED YES)
