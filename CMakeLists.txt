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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(AVCODEC  REQUIRED IMPORTED_TARGET libavcodec)
pkg_check_modules(AVUTIL   REQUIRED IMPORTED_TARGET libavutil)
pkg_check_modules(SWSCALE  REQUIRED IMPORTED_TARGET libswscale)

set(IR_COMP "ircam_component")
set(IR_ENC_COMP "ircam_enc_component")

set(IR_NODE "ircam_node")

# ------- IRCAM RAW COMPONENT --------
add_library(${IR_COMP} SHARED
  src/ircam/v4l2_capture.cpp
  src/ircam/ircam_node.cpp
  src/ircam/ircam_component.cpp  
)
target_include_directories(${IR_COMP} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)
ament_target_dependencies(${IR_COMP}
  rclcpp
  rclcpp_components
  sensor_msgs
)
rclcpp_components_register_node(${IR_COMP}
  PLUGIN "ir_v4l2_camera::IrCameraComponent"
  EXECUTABLE ircam_component_node
)
install(
  TARGETS ${IR_COMP}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ------- IRCAM H264 COMPONENT --------
add_library(${IR_ENC_COMP} SHARED
  src/ircam/ircam_h264_node.cpp
  src/ircam/ircam_h264_component.cpp
)
target_include_directories(${IR_ENC_COMP} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)
ament_target_dependencies(${IR_ENC_COMP}
  rclcpp
  rclcpp_components
  sensor_msgs
)
target_link_libraries(${IR_ENC_COMP}
  PkgConfig::AVCODEC
  PkgConfig::AVUTIL
  PkgConfig::SWSCALE
)
rclcpp_components_register_node(${IR_ENC_COMP}
  PLUGIN "ir_v4l2_camera::IrCameraH264Component"
  EXECUTABLE ircam_h264_component_node
)
install(
  TARGETS ${IR_ENC_COMP}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


# Standalone executable (no composition)
# add_executable(${IR_NODE}
#   src/ircam/v4l2_capture.cpp
#   src/ircam/ircam_node.cpp
#   src/ircam/ircam_exec.cpp
# )
# target_include_directories(${IR_NODE} PUBLIC
#   include
# )
# ament_target_dependencies(${IR_NODE}
#   rclcpp
#   sensor_msgs
# )
# install(
#   TARGETS ${IR_NODE} voxl_h264_decoder voxl_h265_decoder # standalone
#   DESTINATION lib/${PROJECT_NAME}
# )



# Build the h264 node
add_executable(voxl_h264_decoder src/voxl_h264_decoder.cpp)

ament_target_dependencies(voxl_h264_decoder
  rclcpp
  sensor_msgs
)

target_link_libraries(voxl_h264_decoder
  PkgConfig::LIBAV)

# Build the h265 node
add_executable(voxl_h265_decoder src/voxl_h265_decoder.cpp)

ament_target_dependencies(voxl_h265_decoder
  rclcpp
  sensor_msgs
)

target_link_libraries(voxl_h265_decoder
  PkgConfig::LIBAV)



# # Ircam H264 encoding republisher
# add_executable(ircam_h264_enc
#   src/ircam/ircam_h264_enc.cpp
# )
# target_include_directories(ircam_h264_enc PRIVATE
#   ${AVCODEC_INCLUDE_DIRS}
#   ${AVUTIL_INCLUDE_DIRS}
#   ${SWSCALE_INCLUDE_DIRS}
# )
# ament_target_dependencies(ircam_h264_enc
#   rclcpp
#   sensor_msgs
# )
# target_link_libraries(ircam_h264_enc
#   PkgConfig::AVCODEC
#   PkgConfig::AVUTIL
#   PkgConfig::SWSCALE
# )
# install(
#   TARGETS ircam_h264_enc
#   DESTINATION lib/${PROJECT_NAME}
# )



# General install...
install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()