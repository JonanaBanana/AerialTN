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

cd orbslam3_ros2/vocabulary/
tar -xvf ORBVoc.txt.tar.gz
sudo rm -r ORBVoc.txt.tar.gz

cd ~/ros2_ws/

colcon build --symlink-install --packages-select orbslam3