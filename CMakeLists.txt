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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(IR_COMP "ir_v4l2_component")
set(IR_NODE "ircam_node")

# Shared library (composable component)
add_library(${IR_COMP} SHARED
  src/v4l2_capture.cpp
  src/ircam_node.cpp
  src/ircam_component.cpp  
)

target_include_directories(${IR_COMP} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>  
)

ament_target_dependencies(${IR_COMP}
  rclcpp
  rclcpp_components
  sensor_msgs
  std_msgs
)

# Register the component so "ros2 component load" can find it
rclcpp_components_register_node(${IR_COMP}
  PLUGIN "ir_v4l2_camera::IrCameraComponent"
  EXECUTABLE ir_camera_component_node
)

# Standalon executable (no composition)
add_executable(${IR_NODE}
  src/v4l2_capture.cpp
  src/ircam_node.cpp
  src/ircam_exec.cpp
)

target_include_directories(${IR_NODE}
  include
)

ament_target_dependencies(${IR_NODE}
  rclcpp
  sensor_msgs
  std_msgs
)

# Install
install(
  TARGETS ${IR_COMP} # composable
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ${IR_NODE} # standalone
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()