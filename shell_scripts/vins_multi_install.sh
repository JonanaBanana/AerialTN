#!/bin/bash

cd ~/ros2_ws/src/

git clone https://github.com/JonanaBanana/VINS-Multi-ROS2.git vins_multi

cd vins_multi/

chmod +x setup_script.sh

./setup_script.sh

wait

cd ../..

colcon build