#!/bin/bash

cd ~/ros2_ws/src/

git clone https://github.com/JonanaBanana/rovtio_ros2.git

git clone https://github.com/JonanaBanana/rovtio_interfaces_ros2.git

cd ..

colcon build --packages-select rovtio rovtio_interfaces_ros2