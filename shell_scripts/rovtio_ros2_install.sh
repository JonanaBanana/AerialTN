#!/bin/bash

cd ~/ros2_ws/src/

git clone https://github.com/JonanaBanana/rovtio_ros2.git

cd rovtio_ros2/

git submodule update --init --recursive

cd ~/ros2_ws/

colcon build --symlink-install