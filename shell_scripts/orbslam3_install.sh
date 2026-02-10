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

git clone https://gitlab.com/akbedaka/orb_slam3_ros2.git orbslam3_ros2

cd ..

colcon build --symlink-install --packages-select orbslam3