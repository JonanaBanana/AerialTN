#!/bin/bash

#obslam 3 base installer
cd ros2_ws/src/
git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
cd ORB_SLAM3/
chmod +x build.sh
./build.sh
cd ~/thirdparty/ORB_SLAM3/Thirdparty/Sophus/build/
sudo make install