#!/bin/bash
source venv-orbslam3/bin/activate # uncomment if not using virtual environment
wait

#build the ros2 wrapper for orbslam3
cd ~/ros2_ws/src/
wait

git clone https://github.com/JonanaBanana/ORB_SLAM3_ROS2.git orbslam3_ros2
wait

cd ~/ros2_ws/
wait

colcon build --symlink-install --packages-select orbslam3