cmake_minimum_required(VERSION 3.14.4)
project(aerial_tn)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# FFmpeg via pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
  libavcodec
  libavutil
  libswscale)

# Build the h264 node
add_executable(voxl_h264_decoder src/voxl_h264_decoder.cpp)

ament_target_dependencies(voxl_h264_decoder
  rclcpp
  sensor_msgs
  std_msgs)

target_link_libraries(voxl_h264_decoder
  PkgConfig::LIBAV)

# Build the h265 node
add_executable(voxl_h265_decoder src/voxl_h265_decoder.cpp)

ament_target_dependencies(voxl_h265_decoder
  rclcpp
  sensor_msgs
  std_msgs)

target_link_libraries(voxl_h265_decoder
  PkgConfig::LIBAV)

install(TARGETS voxl_h264_decoder voxl_h265_decoder
DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()