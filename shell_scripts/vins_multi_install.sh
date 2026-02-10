#!/bin/bash

mkdir -p ~/vins_multi_ws/src

cd ~/vins_multi_ws/src/

git clone https://github.com/HKUST-Aerial-Robotics/VINS-Multi.git -b multi-ver

cd ~/vins_multi_ws/src/VINS-Multi/

cmake -S core -B core/build -DCMAKE_BUILD_TYPE=Release
cmake --build core/build -j$(nproc)
cmake --install core/build --prefix core/install

cd ros2/

colcon build \
     --base-paths src \
     --packages-select vins_estimator_ros2 \
     --build-base build_ros2_debug \
     --install-base install_ros2_debug \
     --cmake-args \
       -DCMAKE_BUILD_TYPE=RelWithDebInfo \
       -DCMAKE_CXX_FLAGS="-O0 -g3" \
       -DCMAKE_PREFIX_PATH=/home/${USER}/vins_multi_ws/src/VINS-Multi/core/install