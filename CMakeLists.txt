cmake_minimum_required(VERSION 3.8)
project(aerial_tn)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# FFmpeg via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec
  libavutil
  libswscale)

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC  REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL   REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE  REQUIRED IMPORTED_TARGET libswscale)

# Build the h264 node
add_executable(voxl_h264_decoder src/voxl_h264_decoder.cpp)

ament_target_dependencies(voxl_h264_decoder
  rclcpp
  sensor_msgs
)

target_link_libraries(voxl_h264_decoder
  PkgConfig::LIBAV)

install(
  TARGETS voxl_h264_decoder # standalone
  DESTINATION lib/${PROJECT_NAME}
)

# General install...
#install(
#  DIRECTORY include/
#  DESTINATION include
#)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()