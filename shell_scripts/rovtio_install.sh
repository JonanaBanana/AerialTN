#!/bin/bash

cd ~/ros2_ws/src/

git clone https://github.com/JonanaBanana/rovtio_ros2.git

git clone https://github.com/JonanaBanana/rovtio_interfaces_ros2.git

cd ..

colcon build --packages-select rovtio rovtio_interfaces_ros2 --cmake-args -DCMAKE_CXX_FLAGS="-w" 

#-DCMAKE_CXX_FLAGS="-w" just means do not print warnings, only errors. rovtio prints a lot of (harmless) warnings if you do not add this flag, but should still build without issues.
