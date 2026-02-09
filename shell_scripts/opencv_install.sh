#!/bin/bash
source venv-orbslam3/bin/activate # uncomment if not using virtual environment
wait

## install OpenCV with OpenCV_contrib
cd ~
wait

mkdir thirdparty
wait

cd ~/thirdparty/
wait

git clone https://github.com/opencv/opencv
wait

git -C opencv checkout 4.12.0
wait

git clone https://github.com/opencv/opencv_contrib
wait

git -C opencv_contrib checkout 4.12.0
wait

mkdir build
wait

cd build
wait

cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib/modules ../opencv
wait

make -j $(nproc)
wait

sudo make install -j $(nproc)