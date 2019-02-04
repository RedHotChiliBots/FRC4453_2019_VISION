cmake_minimum_required (VERSION 2.6)
project (Vision)
cmake_policy(SET CMP0069 NEW)

include(CheckIPOSupported)

check_ipo_supported(RESULT ipo_avail)

# Project options
set(SYSTEMD_UNIT_DIR "lib/systemd/system/" CACHE PATH "Where to install systemd unit to.")

# Set default for prefix.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "..." FORCE)
endif()

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
if(ipo_avail)
  set(LIBUSB_CFLAGS "${CMAKE_C_FLAGS} -flto")
  set(LIBUSB_CXXFLAGS "${CMAKE_CXX_FLAGS} -flto")
else()
  set(LIBUSB_CFLAGS "${CMAKE_C_FLAGS}")
  set(LIBUSB_CXXFLAGS "${CMAKE_CXX_FLAGS}")
endif()

include(ExternalProject)
ExternalProject_Add(
  libusb
  GIT_REPOSITORY https://github.com/libusb/libusb.git
  CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-shared --disable-udev --host=${GCC_TARGET_TRIPLE} "CFLAGS=${LIBUSB_CFLAGS}" "CXXFLAGS=${LIBUSB_CXXFLAGS}"
  BUILD_COMMAND make "CFLAGS=${LIBUSB_CFLAGS}" "CXXFLAGS=${LIBUSB_CXXFLAGS}"
  BUILD_BYPRODUCTS <INSTALL_DIR>/lib/libusb-1.0.a
)

ExternalProject_Add_Step(
  libusb
  bootstrap
  COMMAND ./bootstrap.sh
  WORKING_DIRECTORY <SOURCE_DIR>
  DEPENDEES update
  DEPENDERS configure
  BYPRODUCTS <SOURCE_DIR>/configure
  ALWAYS 0
)

ExternalProject_Get_Property(libusb install_dir)

# Pixy library
add_library(pixy2usb libs/pixy2/src/common/src/chirp.cpp libs/pixy2/src/host/libpixyusb2/src/libpixyusb2.cpp libs/pixy2/src/host/libpixyusb2/src/usblink.cpp libs/pixy2/src/host/libpixyusb2/src/util.cpp)
add_dependencies(pixy2usb libusb)
target_include_directories(pixy2usb PUBLIC libs/pixy2/src/common/inc libs/pixy2/src/host/libpixyusb2/include libs/pixy2/src/host/arduino/libraries/Pixy2 ${install_dir}/include/libusb-1.0)
target_link_libraries(pixy2usb ${install_dir}/lib/libusb-1.0.a)

# Main binary
add_executable(Vision main.cpp geom.hpp cammath.hpp)
target_include_directories(Vision PUBLIC libs/pixy2/src/host/libpixyusb2/include libs/gcem/include libs/allwpilib/ntcore/src/main/native/include)
add_dependencies(Vision libusb)
target_link_libraries(Vision pixy2usb ntcore)
set_property(TARGET Vision PROPERTY CXX_STANDARD 17)
set_property(TARGET Vision PROPERTY CXX_STANDARD_REQUIRED YES)

if(ipo_avail)
  message(STATUS "IPO Enabled.")
  set_target_properties(Vision pixy2usb ntcore PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()


install(TARGETS Vision
  RUNTIME 
  DESTINATION bin
  COMPONENT Vision)

install(FILES ${CMAKE_SOURCE_DIR}/vision.service 
  DESTINATION ${SYSTEMD_UNIT_DIR}
  COMPONENT Vision)

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_CONTACT "Conner Ebbinghaus")
set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE armhf)
set(CPACK_DEB_COMPONENT_INSTALL ON)
set(CPACK_ARCHIVE_COMPONENT_INSTALL ON)
set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_COMPONENT_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_SET_DESTDIR ON)
SET(CPACK_STRIP_FILES ON)
set(CPACK_COMPONENTS_ALL Vision)

set(CPACK_COMPONENT_VISION_DISPLAY_NAME "Vision")
set (CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

include(CPack)
