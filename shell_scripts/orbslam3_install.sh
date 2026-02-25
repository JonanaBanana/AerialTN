#!/bin/bash
#obslam 3 installer
cd ~/thirdparty
wait

git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
wait

cd ORB_SLAM3/
wait

chmod +x build.sh
wait

./build.sh
wait

cd ~/ros2_ws/src/

git clone https://github.com/JonanaBanana/ORB_SLAM3_ROS2.git orbslam3_ros2

cd ..

colcon build --packages-select orbslam3