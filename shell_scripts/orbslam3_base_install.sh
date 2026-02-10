#!/bin/bash

#obslam 3 base installer
cd ros2_ws/src/
wait

git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
wait

cd ORB_SLAM3/
wait

chmod +x build.sh
wait

./build.sh
wait

cd ~/thirdparty/ORB_SLAM3/Thirdparty/Sophus/build/
wait

sudo make install