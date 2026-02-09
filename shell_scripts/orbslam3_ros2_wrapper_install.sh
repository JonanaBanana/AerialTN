#!/bin/bash

#build the ros2 wrapper for orbslam3
cd ~/ros2_ws/src/
git clone https://github.com/JonanaBanana/ORB_SLAM3_ROS2.git orbslam3_ros2
cd ~/ros2_ws/
colcon build --symlink-install --packages-select orbslam3