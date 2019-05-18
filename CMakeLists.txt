cmake_minimum_required (VERSION 2.6)
project (Vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

#cmake_policy(SET CMP0069 NEW)

#include(CheckIPOSupported)

# check_ipo_supported(RESULT ipo_avail)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/libs/cotire/CMake")
include(cotire)

# Project options
set(SYSTEMD_UNIT_DIR "lib/systemd/system/" CACHE PATH "Where to install systemd unit to.")

# Set default for prefix.
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "..." FORCE)
endif()


if(MINGW) 
  add_compile_definitions(_WIN32_WINNT=0x0A00)
endif()

# Options for wpilib and opencv.
set(WITHOUT_JAVA ON CACHE BOOL "don't include java and JNI in the build" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "build with shared libs (needed for JNI)" FORCE)
set(WITHOUT_CSCORE ON CACHE BOOL "Don't build cscore (removes OpenCV requirement)" FORCE)
set(WITHOUT_ALLWPILIB ON CACHE BOOL "Don't build allwpilib (removes OpenCV requirement)" FORCE)
set(WITH_OPENCL OFF CACHE BOOL "" FORCE)
#set(WITH_OPENNI ON CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_PERF_TESTS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)

# Add wpilib.
add_subdirectory(libs/allwpilib)
add_subdirectory(libs/opencv)

file(GLOB opencv_module_include_dirs libs/opencv/modules/*/include )

# Get our target triple (needed to configure libusb).
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine OUTPUT_VARIABLE GCC_TARGET_TRIPLE)
string(STRIP ${GCC_TARGET_TRIPLE} GCC_TARGET_TRIPLE)
message(STATUS "Target triple: ${GCC_TARGET_TRIPLE}")
endif()

# Add libusb project.
include(ExternalProject)

if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
if(MINGW)
ExternalProject_Add(
  libusb
  URL https://github.com/libusb/libusb/releases/download/v1.0.22/libusb-1.0.22.7z
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(libusb source_dir)
link_directories(${source_dir}/MinGW64/static)
include_directories(${source_dir}/include/libusb-1.0)
else()
ExternalProject_Add(
  libusb
  GIT_REPOSITORY https://github.com/libusb/libusb.git
  UPDATE_COMMAND ""
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
link_directories(${install_dir}/lib)
include_directories(${install_dir}/include/libusb-1.0)
endif()
elseif(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
ExternalProject_Add(
  libusb
  GIT_REPOSITORY https://github.com/libusb/libusb.git
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND <SOURCE_DIR>/configure --prefix=<INSTALL_DIR> --disable-shared --disable-udev "CFLAGS=${LIBUSB_CFLAGS}" "CXXFLAGS=${LIBUSB_CXXFLAGS}" "CC=${CMAKE_C_COMPILER}" "CXX=${CMAKE_CXX_COMPILER}"
  BUILD_COMMAND make "CFLAGS=${LIBUSB_CFLAGS}" "CXXFLAGS=${LIBUSB_CXXFLAGS}"  "CC=${CMAKE_C_COMPILER}" "CXX=${CMAKE_CXX_COMPILER"
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
link_directories(${install_dir}/lib)
include_directories(${install_dir}/include/libusb-1.0)

else()
ExternalProject_Add(
  libusb
  GIT_REPOSITORY https://github.com/libusb/libusb.git
  UPDATE_COMMAND ""
  CONFIGURE_COMMAND ""
  BUILD_COMMAND msbuild.exe /p:Platform=x64 msvc/libusb_static_2017.vcxproj
  BUILD_IN_SOURCE 1
  INSTALL_COMMAND ""
)

ExternalProject_Get_Property(libusb source_dir)
link_directories(${source_dir}/x64/Debug/lib)
include_directories(${source_dir}/libusb)
endif()

# Pixy library
add_library(pixy2usb libs/pixy2/src/common/src/chirp.cpp libs/pixy2/src/host/libpixyusb2/src/libpixyusb2.cpp libs/pixy2/src/host/libpixyusb2/src/usblink.cpp libs/pixy2/src/host/libpixyusb2/src/util.cpp)
add_dependencies(pixy2usb libusb)
target_include_directories(pixy2usb PUBLIC libs/pixy2/src/common/inc libs/pixy2/src/host/libpixyusb2/include libs/pixy2/src/host/arduino/libraries/Pixy2)

# Main binary
add_executable(Vision src/main.cpp include/cammath.hpp include/transform.hpp include/conversion.hpp src/pixyfinder.cpp include/pixyfinder.hpp src/connectionwaiter.cpp include/connectionwaiter.hpp src/visionproc.cpp include/visionproc.hpp)
target_include_directories(Vision PUBLIC include libs/pixy2/src/host/libpixyusb2/include libs/gcem/include libs/allwpilib/ntcore/src/main/native/include libs/eigen libs/spdlog/include)
add_dependencies(Vision libusb)
target_link_libraries(Vision pixy2usb opencv_core opencv_imgproc opencv_features2d opencv_calib3d ntcore libusb-1.0${CMAKE_STATIC_LIBRARY_SUFFIX} )
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
target_link_libraries(Vision atomic -static-libstdc++ -static-libgcc)
endif()
if(MINGW)
target_link_libraries(Vision ws2_32)
endif()
set_property(TARGET Vision PROPERTY CXX_STANDARD 17)
set_property(TARGET Vision PROPERTY CXX_STANDARD_REQUIRED YES)
target_include_directories(Vision PUBLIC libs/opencv/include ${opencv_module_include_dirs} ${CMAKE_BINARY_DIR})
add_dependencies(Vision opencv_modules)
set_target_properties(Vision PROPERTIES
    COTIRE_PREFIX_HEADER_IGNORE_PATH "${CMAKE_SOURCE_DIR}"
    COTIRE_PREFIX_HEADER_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/libs")

# add_executable(VisionKinect src/main_kinect.cpp src/connectionwaiter.cpp include/connectionwaiter.hpp src/visionproc.cpp include/visionproc.hpp)
# target_include_directories(VisionKinect PUBLIC include libs/gcem/include libs/allwpilib/ntcore/src/main/native/include libs/eigen libs/spdlog/include)
# target_link_libraries(VisionKinect opencv_core opencv_imgproc opencv_features2d opencv_calib3d ntcore)

# if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
# target_link_libraries(VisionKinect atomic -static-libstdc++ -static-libgcc)
# endif()
# if(MINGW)
# target_link_libraries(VisionKinect ws2_32)
# endif()
# set_property(TARGET VisionKinect PROPERTY CXX_STANDARD 17)
# set_property(TARGET VisionKinect PROPERTY CXX_STANDARD_REQUIRED YES)
# target_include_directories(VisionKinect PUBLIC libs/opencv/include ${opencv_module_include_dirs} ${CMAKE_BINARY_DIR})
# add_dependencies(VisionKinect opencv_modules)
# set_target_properties(VisionKinect PROPERTIES
#     COTIRE_PREFIX_HEADER_IGNORE_PATH "${CMAKE_SOURCE_DIR}"
#     COTIRE_PREFIX_HEADER_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/libs")

add_executable(VisionTest src/main.cpp include/cammath.hpp include/transform.hpp include/conversion.hpp src/pixyfinder.cpp include/pixyfinder.hpp src/connectionwaiter.cpp include/connectionwaiter.hpp src/visionproc.cpp include/visionproc.hpp)
target_include_directories(VisionTest PUBLIC include libs/pixy2/src/host/libpixyusb2/include libs/gcem/include libs/allwpilib/ntcore/src/main/native/include libs/eigen libs/spdlog/include)
add_dependencies(VisionTest libusb)
target_link_libraries(VisionTest pixy2usb opencv_core opencv_imgproc opencv_features2d opencv_calib3d ntcore libusb-1.0${CMAKE_STATIC_LIBRARY_SUFFIX} )
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
target_link_libraries(VisionTest atomic -static-libstdc++ -static-libgcc)
endif()
if(MINGW)
target_link_libraries(VisionTest ws2_32)
endif()
set_property(TARGET VisionTest PROPERTY CXX_STANDARD 17)
set_property(TARGET VisionTest PROPERTY CXX_STANDARD_REQUIRED YES)
target_compile_definitions(VisionTest PUBLIC DEV_TEST)
target_include_directories(VisionTest PUBLIC libs/opencv/include ${opencv_module_include_dirs} ${CMAKE_BINARY_DIR})
add_dependencies(VisionTest opencv_modules)

set_target_properties(VisionTest PROPERTIES
    COTIRE_PREFIX_HEADER_IGNORE_PATH "${CMAKE_SOURCE_DIR}"
    COTIRE_PREFIX_HEADER_INCLUDE_PATH "${CMAKE_SOURCE_DIR}/libs")

#cotire(Vision VisionTest)

add_executable(PixyRec src/pixyrec.cpp)
target_include_directories(PixyRec PUBLIC libs/pixy2/src/host/libpixyusb2/include)
add_dependencies(PixyRec libusb)
target_link_libraries(PixyRec pixy2usb opencv_core opencv_imgproc opencv_videoio libusb-1.0${CMAKE_STATIC_LIBRARY_SUFFIX} )
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "GNU")
target_link_libraries(PixyRec atomic -static-libstdc++ -static-libgcc)
endif()
set_property(TARGET PixyRec PROPERTY CXX_STANDARD 17)
set_property(TARGET PixyRec PROPERTY CXX_STANDARD_REQUIRED YES)
target_include_directories(PixyRec PUBLIC libs/opencv/include ${opencv_module_include_dirs} ${CMAKE_BINARY_DIR})
add_dependencies(PixyRec opencv_modules)



# if(ipo_avail)
#   message(STATUS "IPO Enabled.")
#   set_target_properties(Vision pixy2usb ntcore PROPERTIES INTERPROCEDURAL_OPTIMIZATION TRUE)
# endif()

#Tests
add_executable(CammathTest test/test.cpp test/test.hpp test/cammath.cpp include/cammath.hpp)
target_include_directories(CammathTest PUBLIC include libs/gcem/include test libs/eigen)
set_property(TARGET CammathTest PROPERTY CXX_STANDARD 17)
set_property(TARGET CammathTest PROPERTY CXX_STANDARD_REQUIRED YES)
add_test(cammath CammathTest)

add_executable(TransformTest test/test.cpp test/test.hpp test/transform.cpp include/transform.hpp include/cammath.hpp)
target_include_directories(TransformTest PUBLIC include libs/gcem/include test libs/eigen)
set_property(TARGET TransformTest PROPERTY CXX_STANDARD 17)
set_property(TARGET TransformTest PROPERTY CXX_STANDARD_REQUIRED YES)
add_test(transform TransformTest)

add_executable(VisionprocTest test/test.cpp test/test.hpp test/visionproc.cpp include/visionproc.hpp src/visionproc.cpp )
target_include_directories(VisionprocTest PUBLIC include libs/gcem/include test libs/eigen )
target_link_libraries(VisionprocTest opencv_core opencv_imgproc opencv_features2d opencv_calib3d)
set_property(TARGET VisionprocTest PROPERTY CXX_STANDARD 17)
set_property(TARGET VisionprocTest PROPERTY CXX_STANDARD_REQUIRED YES)
target_include_directories(VisionprocTest PUBLIC libs/opencv/include ${opencv_module_include_dirs} ${CMAKE_BINARY_DIR})
add_dependencies(VisionprocTest opencv_modules)
add_test(visionproc VisionprocTest)


install(TARGETS Vision
  RUNTIME 
  DESTINATION bin
  COMPONENT Vision)

install(FILES ${CMAKE_SOURCE_DIR}/misc/vision.service 
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
set(CPACK_STRIP_FILES ON)
set(CPACK_COMPONENTS_ALL Vision)

set(CPACK_COMPONENT_VISION_DISPLAY_NAME "Vision")
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

include(CPack)
